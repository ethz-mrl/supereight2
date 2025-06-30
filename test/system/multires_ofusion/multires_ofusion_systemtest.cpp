/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <config.hpp>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <reader.hpp>
#include <se/common/filesystem.hpp>
#include <se/supereight.hpp>

extern int my_argc;
extern char** my_argv;

int my_argc;
char** my_argv;
std::string tmp;

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    my_argc = argc;
    my_argv = argv;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
        exit(2);
    }

    // Create a temporary directory for all the tests.
    tmp = stdfs::temp_directory_path() / stdfs::path("supereight_test_results");
    stdfs::create_directories(tmp);

    return RUN_ALL_TESTS();
}



TEST(MultiResOFusionSystemTest, GetFieldInterpolation)
{
    const std::string config_filename(my_argv[1]);
    se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera> config(config_filename);
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

    // Output files in a temporary directory.
    config.app.mesh_path = std::string(tmp + "/meshes");
    config.app.slice_path = std::string(tmp + "/meshes");
    config.app.structure_path = std::string(tmp + "/meshes");
    config.app.log_file = std::string(tmp + "/log.tsv");
    stdfs::create_directories(config.app.mesh_path);
    stdfs::create_directories(config.app.slice_path);
    stdfs::create_directories(config.app.structure_path);

    // Create a pinhole camera and downsample the intrinsics
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());

    // Set pose to identity
    const Eigen::Isometry3f T_WS = Eigen::Isometry3f::Identity();

    // Set depth image
    for (int y = 0; y < input_img_res.y(); y++) {
        for (int x = 0; x < input_img_res.x(); x++) {
            input_depth_img(x, y) = 4.f;
        }
    }

    se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

    se::MapIntegrator integrator(map);

    const int max_frame = 1;
    for (int frame = 0; frame < max_frame; frame++) {
        integrator.integrateDepth(
            frame, se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});
    }

    map.saveFieldSlices(config.app.slice_path + "/test-field-interp-slice-x.vtk",
                        config.app.slice_path + "/test-field-interp-slice-y.vtk",
                        config.app.slice_path + "/test-field-interp-slice-z.vtk",
                        T_WS.translation());
    map.getOctree().saveStructure(config.app.structure_path + "/test-field-interp-structure_"
                                  + std::to_string(max_frame) + ".ply");

    Eigen::Vector3f point_W;
    std::optional<se::field_t> field_value;

    // Node neighbours different size
    const Eigen::Vector3i voxel_coord_free_1(639, 511, 799);
    map.voxelToPoint(voxel_coord_free_1, point_W);
    field_value = map.interpField(point_W);
    EXPECT_FLOAT_EQ(-5.015, *field_value);

    // All inside node
    const Eigen::Vector3i voxel_coord_free_2(600, 520, 811);
    map.voxelToPoint(voxel_coord_free_2, point_W);
    field_value = map.interpField(point_W);
    EXPECT_FLOAT_EQ(-5.015, *field_value);

    // Node and block neighbours
    const Eigen::Vector3i voxel_coord_free_3(575, 511, 615);
    map.voxelToPoint(voxel_coord_free_3, point_W);
    field_value = map.interpField(point_W);
    EXPECT_FLOAT_EQ(-5.015, *field_value);
}



TEST(MultiResOFusionSystemTest, GetField)
{
    const std::string config_filename(my_argv[1]);
    se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera> config(config_filename);
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

    // Output files in a temporary directory.
    config.app.mesh_path = std::string(tmp + "/meshes");
    config.app.slice_path = std::string(tmp + "/meshes");
    config.app.structure_path = std::string(tmp + "/meshes");
    config.app.log_file = std::string(tmp + "/log.tsv");
    stdfs::create_directories(config.app.mesh_path);
    stdfs::create_directories(config.app.slice_path);
    stdfs::create_directories(config.app.structure_path);

    // Create a pinhole camera and downsample the intrinsics
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());

    // Set pose to identity
    const Eigen::Isometry3f T_WS = Eigen::Isometry3f::Identity();

    // Set depth image
    for (int y = 0; y < input_img_res.y(); y++) {
        for (int x = 0; x < input_img_res.x(); x++) {
            input_depth_img(x, y) = (x < input_img_res.x() / 2) ? 2.f : 4.f;
        }
    }

    se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

    se::MapIntegrator integrator(map);

    const int max_frame = 1;
    for (int frame = 0; frame < max_frame; frame++) {
        integrator.integrateDepth(
            frame, se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});
    }

    const Eigen::Vector3i voxel_coord_unknown_1(688, 500, 933);
    const Eigen::Vector3i voxel_coord_unknown_2(256, 166, 338);
    const Eigen::Vector3i voxel_coord_free_1(578, 500, 737);
    const Eigen::Vector3i voxel_coord_free_2(477, 500, 618);
    Eigen::Vector3f point_W;

    se::OccupancyData data;

    map.saveFieldSlices(config.app.slice_path + "/test-field-slice-x.vtk",
                        config.app.slice_path + "/test-field-slice-y.vtk",
                        config.app.slice_path + "/test-field-slice-z.vtk",
                        T_WS.translation());

    map.getOctree().saveStructure(config.app.structure_path + "/test-field-structure_"
                                  + std::to_string(max_frame) + ".ply");

    map.voxelToPoint(voxel_coord_unknown_1, point_W);
    data = map.getData(point_W);
    EXPECT_EQ(se::OccupancyData().field.occupancy, data.field.occupancy);

    map.voxelToPoint(voxel_coord_unknown_2, point_W);
    data = map.getData(point_W);
    EXPECT_EQ(se::OccupancyData().field.occupancy, data.field.occupancy);

    map.voxelToPoint(voxel_coord_free_1, point_W);
    data = map.getData(point_W);
    EXPECT_FLOAT_EQ(-5.015, data.field.occupancy);

    map.voxelToPoint(voxel_coord_free_2, point_W);
    data = map.getData(point_W);
    EXPECT_FLOAT_EQ(-5.015, data.field.occupancy);
}



TEST(MultiResOFusionSystemTest, GetMaxField)
{
    const std::string config_filename(my_argv[1]);
    se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera> config(config_filename);
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

    // Output files in a temporary directory.
    config.app.mesh_path = std::string(tmp + "/meshes");
    config.app.slice_path = std::string(tmp + "/meshes");
    config.app.structure_path = std::string(tmp + "/meshes");
    config.app.log_file = std::string(tmp + "/log.tsv");
    stdfs::create_directories(config.app.mesh_path);
    stdfs::create_directories(config.app.slice_path);
    stdfs::create_directories(config.app.structure_path);

    // Create a pinhole camera and downsample the intrinsics
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());

    // Set pose to identity
    const Eigen::Isometry3f T_WS = Eigen::Isometry3f::Identity();

    // Set depth image
    for (int y = 0; y < input_img_res.y(); y++) {
        for (int x = 0; x < input_img_res.x(); x++) {
            input_depth_img(x, y) = (x < input_img_res.x() / 2) ? 2.f : 4.f;
        }
    }

    se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

    se::MapIntegrator integrator(map);

    const int max_frame = 1;
    for (int frame = 0; frame < max_frame; frame++) {
        integrator.integrateDepth(
            frame, se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});
    }

    const Eigen::Vector3i voxel_coord(640, 512, 896);
    const int scale_5 = 5;
    Eigen::Vector3f point_W;

    se::OccupancyData data;

    map.voxelToPoint(voxel_coord, point_W);
    data = map.getMaxData(point_W, scale_5);

    map.saveFieldSlices(config.app.slice_path + "/test-max-field-slice-field-x.vtk",
                        config.app.slice_path + "/test-max-field-slice-field-y.vtk",
                        config.app.slice_path + "/test-max-field-slice-field-z.vtk",
                        T_WS.translation());

    map.saveScaleSlices(config.app.slice_path + "/test-max-field-slice-scale-x.vtk",
                        config.app.slice_path + "/test-max-field-slice-scale-y.vtk",
                        config.app.slice_path + "/test-max-field-slice-scale-z.vtk",
                        T_WS.translation());



    const int max_scale = map.getOctree().getMaxScale();
    for (int scale = 0; scale < max_scale; scale++) {
        map.saveMaxFieldSlices(config.app.slice_path + "/test-max-field-slice-max-field-scale-"
                                   + std::to_string(scale) + "-x.vtk",
                               config.app.slice_path + "/test-max-field-slice-max-field-scale-"
                                   + std::to_string(scale) + "-y.vtk",
                               config.app.slice_path + "/test-max-field-slice-max-field-scale-"
                                   + std::to_string(scale) + "-z.vtk",
                               T_WS.translation(),
                               scale);
    }

    map.getOctree().saveStructure(config.app.structure_path + "/test-max-field-structure_"
                                  + std::to_string(max_frame) + ".ply");
}



TEST(MultiResOFusionSystemTest, DeleteChildren)
{
    const std::string config_filename(my_argv[1]);
    se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera::Config> config(config_filename);
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

    // Output files in a temporary directory.
    config.app.mesh_path = std::string(tmp + "/meshes");
    config.app.slice_path = std::string(tmp + "/meshes");
    config.app.structure_path = std::string(tmp + "/meshes");
    config.app.log_file = std::string(tmp + "/log.tsv");
    stdfs::create_directories(config.app.mesh_path);
    stdfs::create_directories(config.app.slice_path);
    stdfs::create_directories(config.app.structure_path);

    // Create a pinhole camera and downsample the intrinsics
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    se::Image<float> input_noise_depth_img(input_img_res.x(), input_img_res.y());
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());

    // Set pose to identity
    const Eigen::Isometry3f T_WS = Eigen::Isometry3f::Identity();

    // Set depth image
    for (int y = 0; y < input_img_res.y(); y++) {
        for (int x = 0; x < input_img_res.x(); x++) {
            input_noise_depth_img(x, y) =
                (x < input_img_res.x() * 3 / 8 || x > input_img_res.x() * 5 / 8) ? 2.f : 4.f;
        }
    }

    for (int y = 0; y < input_img_res.y(); y++) {
        for (int x = 0; x < input_img_res.x(); x++) {
            input_depth_img(x, y) = 4.f;
        }
    }

    se::MapIntegrator integrator(map);

    const int max_frame = 15;
    for (int frame = 0; frame < max_frame; frame++) {
        se::preprocessor::downsample_depth((frame == 0) ? input_noise_depth_img : input_depth_img,
                                           processed_depth_img);
        integrator.integrateDepth(
            frame, se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});

        const Eigen::Vector3i voxel_coord(471, 512, 807);
        Eigen::Vector3f point_W;

        se::OccupancyData data;

        map.voxelToPoint(voxel_coord, point_W);
        data = map.getData(point_W);

        map.saveFieldSlices(
            config.app.slice_path + "/test-delete-child-slice-" + std::to_string(frame) + "-x.vtk",
            config.app.slice_path + "/test-delete-child-slice-" + std::to_string(frame) + "-y.vtk",
            config.app.slice_path + "/test-delete-child-slice-" + std::to_string(frame) + "-z.vtk",
            T_WS.translation());

        map.getOctree().saveStructure(config.app.structure_path + "/test-delete-child-structure_"
                                      + std::to_string(frame) + ".ply");
    }
}



TEST(MultiResOFusionSystemTest, Raycasting)
{
    // Read the configuration
    const std::string config_filename(my_argv[1]);
    se::Config<se::OccupancyMap<se::Res::Multi>, se::PinholeCamera> config(config_filename);

    // Output files in a temporary directory.
    config.app.mesh_path = std::string(tmp + "/meshes");
    config.app.slice_path = std::string(tmp + "/meshes");
    config.app.structure_path = std::string(tmp + "/meshes");
    config.app.log_file = std::string(tmp + "/log.tsv");
    stdfs::create_directories(config.app.mesh_path);
    stdfs::create_directories(config.app.slice_path);
    stdfs::create_directories(config.app.structure_path);

    // Setup log stream
    std::ofstream log_file_stream;
    log_file_stream.open(config.app.log_file);
    se::perfstats.setFilestream(&log_file_stream);

    // Setup the map
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

    // Setup input images
    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    se::Image<se::RGB> input_colour_img(input_img_res.x(), input_img_res.y());

    // Setup processed images
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());
    se::Image<se::RGB> processed_colour_img(processed_img_res.x(), processed_img_res.y());

    // Setup output images / renders
    se::Image<se::RGBA> output_colour_img(processed_img_res.x(), processed_img_res.y());
    se::Image<se::RGBA> output_depth_img(processed_img_res.x(), processed_img_res.y());
    se::Image<se::RGBA> output_tracking_img(processed_img_res.x(), processed_img_res.y());
    se::Image<se::RGBA> output_volume_img(processed_img_res.x(), processed_img_res.y());

    // Create a pinhole camera and downsample the intrinsics
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    // ========= READER INITIALIZATION  =========
    std::unique_ptr<se::Reader> reader(se::create_reader(config.reader));
    if (!reader) {
        exit(EXIT_FAILURE);
    }

    // Integrated depth at given pose

    int frame = 0;

    se::Image<Eigen::Vector3f> surface_point_cloud_W(processed_img_res.x(), processed_img_res.y());
    se::Image<Eigen::Vector3f> surface_normals_W(processed_img_res.x(), processed_img_res.y());
    se::Image<int8_t> surface_scale(processed_img_res.x(), processed_img_res.y());

    Eigen::Isometry3f T_WS;
    reader->nextData(input_depth_img, input_colour_img, T_WS);

    // Preprocess depth
    const se::Image<size_t> downsample_map =
        se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
    se::image::remap(input_colour_img, processed_colour_img, downsample_map);

    se::MapIntegrator integrator(map);
    integrator.integrateDepth(frame,
                              se::Measurements{se::Measurement{processed_depth_img, sensor, T_WS}});

    se::raycaster::raycast_volume(
        map, sensor, T_WS, surface_point_cloud_W, surface_normals_W, surface_scale);

    se::image::rgb_to_rgba(processed_colour_img, output_colour_img);
    convert_to_output_depth_img(processed_depth_img, output_depth_img.data());
    se::raycaster::render_volume_scale(output_volume_img,
                                       surface_point_cloud_W,
                                       surface_normals_W,
                                       surface_scale,
                                       T_WS.translation());

    map.getOctree().saveStructure(config.app.structure_path + "/test-raycasting-structure_"
                                  + std::to_string(frame) + ".ply");
    map.getOctree().saveStructure(config.app.structure_path + "/test-raycasting-structure_"
                                  + std::to_string(frame) + ".vtk");
    map.getOctree().saveStructure(config.app.structure_path + "/test-raycasting-structure_"
                                  + std::to_string(frame) + ".obj");
    map.saveFieldSlices(
        config.app.slice_path + "/test-raycasting-slice-field-" + std::to_string(frame) + "-x.vtk",
        config.app.slice_path + "/test-raycasting-slice-field-" + std::to_string(frame) + "-y.vtk",
        config.app.slice_path + "/test-raycasting-slice-field-" + std::to_string(frame) + "-z.vtk",
        T_WS.translation());
    map.saveMesh(config.app.mesh_path + "/test-raycasting-mesh_" + std::to_string(frame) + ".ply");
    map.saveMesh(config.app.mesh_path + "/test-raycasting-mesh_" + std::to_string(frame) + ".vtk");
    map.saveMesh(config.app.mesh_path + "/test-raycasting-mesh_" + std::to_string(frame) + ".obj");

    cv::Mat depth_cv_image(
        processed_img_res.y(), processed_img_res.x(), CV_8UC4, output_volume_img.data());
    cv::cvtColor(depth_cv_image, depth_cv_image, cv::COLOR_RGBA2BGRA);
    cv::imwrite((config.app.mesh_path + "/test-raycasting-img.png").c_str(), depth_cv_image);
}
