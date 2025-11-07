/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/common/filesystem.hpp>
#include <se/common/system_utils.hpp>
#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"

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

        // Setup output images / renders
        se::Image<se::RGB> downsampled_colour_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_colour_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_depth_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> output_tracking_img(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> scale_render(processed_img_res.x(), processed_img_res.y());
        se::Image<se::RGBA> colour_render(processed_img_res.x(), processed_img_res.y());

        // ========= Map INITIALIZATION  =========
        // Setup the single-res TSDF map w/ default block size of 8 voxels
        // Custom way of setting up the same map:
        // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Id::Off>, se::Res::Single, 8>
        // See end of map.hpp and data.hpp for more details
        MapType map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera and downsample the intrinsics
        // Supported sensor models {se::PinholeCamera, se::RangeImageLidar}
        const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);
        const se::PinholeCamera colour_sensor(config.sensor);

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

        int frame = 0;
        while (frame != config.app.max_frames) {
            se::perfstats.setIter(frame++);

            TICK("total")

            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            if (config.app.enable_ground_truth || frame == 1) {
                read_ok = reader->nextData(input_depth_img, input_colour_img, T_WB);
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
                            se::Measurement{input_colour_img, colour_sensor, T_WS * T_SSc}});
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
                                              &surface_colour);
            }
            TOCK("raycast")

            // Convert colour, depth and render the volume (if enabled)
            // The volume is only rendered at the set rendering rate
            TICK("render")
            if (config.app.enable_rendering) {
                se::image::remap(input_colour_img, downsampled_colour_img, downsample_map);
                se::image::rgb_to_rgba(downsampled_colour_img, output_colour_img);
                convert_to_output_depth_img(processed_depth_img,
                                            sensor.near_plane,
                                            sensor.far_plane,
                                            output_depth_img.data());
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
                labels.emplace_back(config.app.enable_ground_truth ? "TRACKING OFF" : "TRACKING");
                images.emplace_back(res, CV_8UC4, output_tracking_img.data());
                labels.emplace_back(has_colour ? "COLOUR RENDER" : "NO COLOUR");
                images.emplace_back(res, CV_8UC4, colour_render.data());
                labels.emplace_back("SCALE RENDER");
                images.emplace_back(res, CV_8UC4, scale_render.data());
                // Combine all the images into one, overlay the labels and show it.
                cv::Mat render = se::montage(3, 2, images, labels);
                drawit(reinterpret_cast<se::RGBA*>(render.data),
                       Eigen::Vector2i(render.cols, render.rows));
            }
            TOCK("draw")

            // Save logs, mesh, slices and struct (if enabled)
            TOCK("total")
            const bool last_frame =
                frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
            if ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0)
                || last_frame) {
                if (!config.app.mesh_path.empty()) {
                    stdfs::create_directories(config.app.mesh_path);
                    const auto mesh = map.mesh();
                    se::io::save_mesh(
                        mesh, config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
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
                    map.getOctree().saveStructure(config.app.structure_path + "/struct_"
                                                  + std::to_string(frame) + ".ply");
                }
            }

            se::perfstats.sample("memory usage",
                                 se::system::memory_usage_self() / (1024.0 * 1024.0),
                                 se::PerfStats::MEMORY);
            se::perfstats.writeToFilestream();
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
