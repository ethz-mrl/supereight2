/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 * This is based on main.cpp
 */

#include <queue>

#include <se/common/filesystem.hpp>
#include <se/common/system_utils.hpp>
#include <se/supereight.hpp>
#include <se/map/octree/fetcher.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <nanoflann.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"

template<typename DataT>
bool is_free(const DataT& data)
{
    return get_field(data) < 0.0f;
}

void savePoints(size_t id,
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points,
                std::string save_path,
                std::string type_name) {
  size_t Npoints = points.size();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_ptr->width = Npoints;
  cloud_ptr->height = 1;
  cloud_ptr->is_dense = false;
  cloud_ptr->resize (cloud_ptr->width * cloud_ptr->height);
  size_t tmpCnt = 0;
  for (auto& point: cloud_ptr->points) {
    Eigen::Vector3d point_W = points[tmpCnt].cast<double>();
    point.x = point_W(0);
    point.y = point_W(1);
    point.z = point_W(2);

    if (type_name == "edges") {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
    else if (type_name == "all") {
        point.r = 255;
        point.g = 255;
        point.b = 255;
    }
    tmpCnt ++;
  }  
  std::string saveName = save_path + "/" + type_name + "_" + std::to_string(id) + ".ply";
  pcl::io::savePLYFileASCII(saveName, *cloud_ptr);
}


int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        typedef se::OccupancyColMap<se::Res::Multi> MapType;
        typedef se::OccupancyColMap<>::OctreeType::NodeType NodeType;
        typedef se::OccupancyColMap<>::OctreeType::BlockType BlockType;
        typedef se::OccupancyColMap<>::OctreeType::DataType DataType;

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
        // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off>, se::Res::Single, 8>
        // See end of map.hpp and data.hpp for more details
        MapType map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera and downsample the intrinsics
        // Supported sensor models {se::PinholeCamera, se::OusterLidar}
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

            TICK("DetectSemantics")
            std::queue<const NodeType*> nodes;
            std::queue<const BlockType*> blocks;
            const NodeType* const root = static_cast<const NodeType*>(map.getOctree().getRoot());
            nodes.push(root);
            const int detect_rate = 100;
            if (frame % detect_rate == 0) {
                while (!nodes.empty()) {
                    const NodeType* const node = nodes.front();
                    nodes.pop();
                    // Test the data of all children are within the minimum and maximum data of the parent.
                    for (int child_idx = 0; child_idx < 8; child_idx++) {
                        const se::OctantBase* const child = node->getChild(child_idx);
                        if (child) {
                            // Get the child min/max data and add it to the appropriate traversal queue.
                            if (child->is_block) {
                                blocks.push(static_cast<const BlockType*>(child));
                            }
                            else {
                                nodes.push(static_cast<const NodeType*>(child));
                            }
                        }
                    }
                }
                std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> threed_edges;
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> threed_fedges;
                std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> test_points;
                while (!blocks.empty()) {
                    const BlockType* block = blocks.front();
                    blocks.pop();
                    const int scale = block->getCurrentScale();
                    if (scale != 0) continue;
                    const int block_size = block->getSize();
                    const Eigen::Vector3i& block_coord = block->coord; // the smallest coordinate in the block.
                    const Eigen::Vector3i block_upper_coord =
                        block_coord + Eigen::Vector3i::Constant(block_size);
                    const int size = se::octantops::scale_to_size(scale);

                    // Test each voxel at the current scale.
                    for (int z = 0; z < block_size; z += size) {
                        for (int y = 0; y < block_size; y += size) {
                            for (int x = 0; x < block_size; x += size) {
                                const Eigen::Vector3i coord = block_coord + Eigen::Vector3i(x,y,z); // global coord.
                                const DataType block_data = block->getData(coord);
                                // 3D edge test for all voxels.
                                if (block_data.field.valid() && !is_free(block_data)) { // observed & occupied
                                    // TMP: Save all tested voxels.
                                    Eigen::Vector3f point_W;
                                    map.voxelToPoint(coord, point_W);
                                    test_points.push_back(point_W);
                                    // Test 26 neighbors
                                    unsigned char num_free = 0;
                                    for (int i = 0; i < 26; i++) {
                                        const Eigen::Vector3i neighbour_coord = 
                                            coord + size*se::fetcher::all_neighbour_offsets.col(i);
                                        const bool neighbour_in_block =
                                            (neighbour_coord.array() >= block_coord.array()).all()
                                            && (neighbour_coord.array() < block_upper_coord.array()).all();
                                        if (neighbour_in_block) {
                                            // If free
                                            if (is_free(block->getData(neighbour_coord))) {
                                                num_free ++;
                                            }
                                        }
                                    }
                                    if (num_free > 11) { // 9(surface) + 2
                                        threed_edges.push_back(coord);
                                        // TMP: for visualizing 3d edges.
                                        Eigen::Vector3f edge_W;
                                        map.voxelToPoint(coord, edge_W);
                                        threed_fedges.push_back(edge_W);
                                    }
                                }
                            }
                        }
                    }
                }
                // TMP: save intermediate mesh & 3d edges.
                map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
                savePoints(frame, threed_fedges, config.app.mesh_path, "edges");
                savePoints(frame, test_points, config.app.mesh_path, "all");
                std::cout << "[" << frame << "] " << threed_edges.size() << " edges3D detected in " << test_points.size() << std::endl;

                // TODO: Detect closed edges.

            }
            TOCK("DetectSemantics")

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
                    map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
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
                                 PerfStats::MEMORY);
            se::perfstats.writeToFilestream();
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
