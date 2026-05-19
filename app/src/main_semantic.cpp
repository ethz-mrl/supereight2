/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <unordered_map>
#include <se/common/filesystem.hpp>
#include <se/common/system_utils.hpp>
#include <se/image/util.hpp>
#include <se/supereight.hpp>
#include <se/common/id.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"


struct VoxelKey {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x &&
               y == other.y &&
               z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        std::size_t h1 = std::hash<int>{}(k.x);
        std::size_t h2 = std::hash<int>{}(k.y);
        std::size_t h3 = std::hash<int>{}(k.z);

        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Convert meter coordinates into voxel coordinates
VoxelKey makeKey(float x, float y, float z, float resolution = 0.01f)
{
    return {
        static_cast<int>(std::round(x / resolution)),
        static_cast<int>(std::round(y / resolution)),
        static_cast<int>(std::round(z / resolution))
    };
}


int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        typedef se::TSDFMap<se::Res::Single> MapType;

        // ========= Config & I/O INITIALIZATION  =========
        const std::string config_filename = argv[1];
        const se::Config<MapType, se::PinholeCamera> config(config_filename);
        std::cout << config;

        // Setup log stream
        std::ofstream log_file_stream;
        log_file_stream.open(config.app.log_file);
        se::perfstats.setFilestream(&log_file_stream);

        // Setup input images
        const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
        se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
        se::Image<se::RGB> input_colour_img(input_img_res.x(), input_img_res.y());

        // Setup processed images
        const Eigen::Vector2i processed_img_res =
            input_img_res / config.app.sensor_downsampling_factor;
        se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::id_t> segment_img(processed_img_res.x(), processed_img_res.y());

        // Setup output images / renders
        se::Image<se::RGB> downsampled_colour_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::id_t> downsampled_segment_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_colour_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_segment_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_depth_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_tracking_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> scale_render(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> colour_render(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> segment_render(processed_img_res.x(), processed_img_res.y());

        // ========= Map INITIALIZATION  =========
        // Setup the single-res TSDF map w/ default block size of 8 voxels
        // Custom way of setting up the same map:
        // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off>, se::Res::Single, 8>
        // See end of map.hpp and data.hpp for more details
        MapType map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera and downsample the intrinsics
        // Supported sensor models {se::PinholeCamera, se::OusterLidar}
        const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);
        const se::PinholeCamera colour_sensor(config.sensor);
        const se::PinholeCamera segment_sensor = colour_sensor;

        // ========= READER INITIALIZATION  =========
        std::unique_ptr<se::Reader> reader(se::create_reader(config.reader));
        if (!reader) {
            return EXIT_FAILURE;
        }
        const bool has_colour = MapType::DataType::col_ == se::Colour::On && reader->hasColour();

        // Setup input, processed and output imgs
        Eigen::Isometry3f T_WB = Eigen::Isometry3f::Identity(); //< Body to world transformation
        Eigen::Isometry3f T_BS = sensor.T_BS;                   //< Sensor to body transformation
        Eigen::Isometry3f T_WS = T_WB * T_BS;                   //< Sensor to world transformation
        // TODO: use the correct T_SSc depending on the dataset
        const Eigen::Isometry3f T_SSc = Eigen::Isometry3f::Identity();
        const Eigen::Isometry3f T_SSs = T_SSc;

        // ========= Tracker & Pose INITIALIZATION  =========
        se::Tracker tracker(map, sensor, config.tracker);

        // ========= Integrator INITIALIZATION  =========
        // The integrator uses a field dependent allocation (TSDF: ray-casting; occupancy: volume-carving)
        // and updating method
        se::MapIntegrator integrator(map);

        // Setup surface pointcloud, normals and scale
        se::Image<Eigen::Vector3f> surface_point_cloud_W(processed_img_res.x(),
                                                         processed_img_res.y());
        se::Image<Eigen::Vector3f> surface_normals_W(processed_img_res.x(), processed_img_res.y());
        se::Image<int8_t> surface_scale(processed_img_res.x(), processed_img_res.y());
        se::Image<se::colour_t> surface_colour(processed_img_res.x(), processed_img_res.y());
        se::Image<se::id_t> surface_segment_id(processed_img_res.x(),
                                                       processed_img_res.y());

        int frame = 0;
        struct DefectPixel{
            int frame_id;
            int u;
            int v;
            // BACKGROUND = 1;
            // BUCKLING_ID = 2;
            // SEAM_CORROSION_ID = 3;
            // EDGE_CORROSION_ID = 4;
            // SPOT_CORROSION_ID = 5;
            id_t defect_id;

            DefectPixel(int frame_id_, int u_, int v_, id_t defect_id_)
                : frame_id(frame_id_), u(u_), v(v_), defect_id(defect_id_) {}
        };
        // 3D defect voxel to defect pixels.
        std::unordered_map<VoxelKey, std::vector<DefectPixel>, VoxelKeyHash> table_2d3d;
        while (frame != config.app.max_frames) {
            se::perfstats.setIter(frame++);

            TICK("total")

            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            if (config.app.enable_ground_truth || frame == 1) {
                read_ok = reader->nextData(input_depth_img, input_colour_img, segment_img, T_WB);
                T_WS = T_WB * T_BS;
            }
            else {
                read_ok = reader->nextData(input_depth_img, input_colour_img);
            }
            if (read_ok != se::ReaderStatus::ok) {
                break;
            }
            TOCK("read")

            // Preprocess depth
            TICK("ds-depth")
            const se::Image<size_t> downsample_map =
                se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
            TOCK("ds-depth")

            // Track pose (if enabled)
            // Initial pose (frame == 0) is initialised with the identity matrix
            TICK("tracking")
            if (!config.app.enable_ground_truth && frame > 1
                && (frame % config.app.tracking_rate == 0)) {
                tracker.track(processed_depth_img, T_WS, surface_point_cloud_W, surface_normals_W);
            }
            se::perfstats.sampleT_WB(T_WB);
            TOCK("tracking")

            // Integrate depth for a given sensor, depth image, pose and frame number
            TICK("integration")
            if (frame % config.app.integration_rate == 0) {
                if (has_colour) {
                    integrator.integrateDepth(
                        frame,
                        se::Measurements{
                            se::Measurement{processed_depth_img, sensor, T_WS},
                            se::Measurement{input_colour_img, colour_sensor, T_WS * T_SSc},
                            se::Measurement{segment_img, segment_sensor, T_WS * T_SSs}});
                }
                else {
                    integrator.integrateDepth(
                        frame,
                        se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});
                }
            }
            TOCK("integration")

            // Raycast from T_MS
            TICK("raycast")
            if (config.app.enable_rendering || !config.app.enable_ground_truth) {
                se::raycaster::raycast_volume(map,
                                              sensor,
                                              T_WS,
                                              surface_point_cloud_W,
                                              surface_normals_W,
                                              surface_scale,
                                              &surface_colour,
                                              &surface_segment_id);
            }
            TOCK("raycast")

            // Convert colour, depth and render the volume (if enabled)
            // The volume is only rendered at the set rendering rate
            TICK("render")
            if (config.app.enable_rendering) {
                se::image::remap(input_colour_img, downsampled_colour_img, downsample_map);
                se::image::rgb_to_rgba(downsampled_colour_img, output_colour_img);
                se::image::remap(segment_img, downsampled_segment_img, downsample_map);
                for (size_t i = 0; i < output_segment_img.size(); i++) {
                    const se::RGB c = se::id_colour(downsampled_segment_img[i]);
                    output_segment_img[i] = se::RGBA{c.r, c.g, c.b, 0xFF};
                }
                se::image::depth_to_rgba(
                    processed_depth_img, output_depth_img, sensor.near_plane, sensor.far_plane);
                tracker.renderTrackingResult(output_tracking_img.data());
                if (frame % config.app.rendering_rate == 0) {
                    se::raycaster::render_volume_scale(scale_render,
                                                       surface_point_cloud_W,
                                                       surface_normals_W,
                                                       surface_scale,
                                                       T_WS.translation());
                    if (has_colour) {
                        se::raycaster::render_volume_colour(colour_render,
                                                            surface_point_cloud_W,
                                                            surface_normals_W,
                                                            surface_colour,
                                                            T_WS.translation());
                        se::raycaster::render_volume(
                            segment_render,
                            surface_point_cloud_W,
                            surface_normals_W,
                            [&surface_segment_id](const size_t pixel_idx) {
                                return se::id_colour(surface_segment_id[pixel_idx]);
                            },
                            T_WS.translation());
                    }
                }
            }
            TOCK("render")

            // Visualise colour, depth, tracking data and the volume render (if enabled)
            TICK("draw")
            if (config.app.enable_gui) {
                // Create vectors of images and labels.
                cv::Size res(processed_img_res.x(), processed_img_res.y());
                std::vector<cv::Mat> images;
                std::vector<std::string> labels;
                labels.emplace_back("INPUT RGB");
                images.emplace_back(res, CV_8UC4, output_colour_img.data());
                labels.emplace_back("INPUT DEPTH");
                images.emplace_back(res, CV_8UC4, output_depth_img.data());
                labels.emplace_back("INPUT SEGMENTS");
                images.emplace_back(res, CV_8UC4, output_segment_img.data());
                labels.emplace_back(has_colour ? "COLOUR RENDER" : "NO COLOUR");
                images.emplace_back(res, CV_8UC4, colour_render.data());
                labels.emplace_back("SCALE RENDER");
                images.emplace_back(res, CV_8UC4, scale_render.data());
                labels.emplace_back("SEGMENT RENDER");
                images.emplace_back(res, CV_8UC4, segment_render.data());
                // Combine all the images into one, overlay the labels and show it.
                cv::Mat render = se::montage(3, 2, images, labels);
                drawit(reinterpret_cast<se::RGBA*>(render.data),
                       Eigen::Vector2i(render.cols, render.rows));
            }
            TOCK("draw")

            // Compute 2D defect to 3D point indexing
            TICK("indexing")
            assert(processed_depth_img.width() == segment_img.width());
            assert(processed_depth_img.height() == segment_img.height());
            const uchar BACKGND_ID = 1;
            const Eigen::Isometry3f T_WSs = T_WS*T_SSs;
            for (int u = 0; u < segment_img.width(); u++) {
                for (int v = 0; v < segment_img.height(); v++) {
                    // If there is a defect (not background).
                    if (segment_img(u,v) != BACKGND_ID) {
                        const Eigen::Vector2f uv(static_cast<float>(u), static_cast<float>(v));
                        const float depth = processed_depth_img(u,v);
                        if (depth <= sensor.far_plane && depth >= sensor.near_plane) {
                            Eigen::Vector3f ray_uv;
                            segment_sensor.model.backProject(uv, &ray_uv);
                            const Eigen::Vector3f p_Ss = ray_uv * depth;
                            const Eigen::Vector3f p_W = T_WSs*p_Ss; // this should be rotation + translation.
                            DefectPixel defect_uv(frame, u, v, segment_img(u,v));
                            VoxelKey voxel_key = makeKey(p_W.x(), p_W.y(), p_W.z(), config.map.res);
                            table_2d3d[voxel_key].push_back(defect_uv);
                        }
                    }
                }
            }
            TOCK("indexing")

            // Save logs, mesh, slices and struct (if enabled)
            TOCK("total")
            const bool last_frame =
                frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
            if ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0)
                || last_frame) {
                if (!config.app.mesh_path.empty()) {
                    stdfs::create_directories(config.app.mesh_path);
                    TICK("meshing (s)")
                    auto mesh = map.mesh();
                    TOCK("meshing (s)")
                    TICK("mesh colouring (s)")
                    se::id::colour_mesh_by_id(mesh, false);
                    TOCK("mesh colouring (s)")
                    TICK("mesh saving (s)")
                    const std::string filename =
                        config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply";
                    se::io::save_mesh(mesh, filename);
                    TOCK("mesh saving (s)")
                    se::perfstats.sample("mesh size",
                                         mesh.size() * sizeof(decltype(mesh)::value_type)
                                             / (1024.0 * 1024.0),
                                         se::PerfStats::MEMORY);
                }
                if (!config.app.slice_path.empty()) {
                    stdfs::create_directories(config.app.slice_path);
                    map.saveFieldSlices(
                        config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                        T_WS.translation());
                }
                if (!config.app.structure_path.empty()) {
                    stdfs::create_directories(config.app.structure_path);
                    map.saveStructure(config.app.structure_path + "/struct_" + std::to_string(frame)
                                      + ".ply");
                }
            }

            se::perfstats.sample("memory usage",
                                 se::system::memory_usage_self() / (1024.0 * 1024.0),
                                 se::PerfStats::MEMORY);
            se::perfstats.writeToFilestream();
        }

        // Print first N values.
        std::cout << "table size = " << table_2d3d.size() << ", voxel resolution = " << config.map.res << std::endl;
        size_t firstN = 0;
        for (const auto& ival : table_2d3d) {
            std::cout << "VoxelKey = " << ival.first.x << ", " << ival.first.y << ", " << ival.first.z << std::endl;
            for (const auto& jval : ival.second) {
                std::cout << "  " << jval.frame_id << ", " << jval.u << ", " << jval.v << ", " << jval.defect_id << std::endl;
            }
            std::cout << "\n";
            if (firstN > 10) {
                break;
            }
            firstN ++;
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}