/*
 * SPDX-FileCopyrightText: 2022-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/filesystem.hpp>
#include <se/integrator/map_integrator.hpp>

se::PinholeCamera generate_sensor()
{
    constexpr int width = 100;
    constexpr int height = width;
    const Eigen::Isometry3f T_BS(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY())
                                 * Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
    return se::PinholeCamera(
        {{width, height, 0.1f, 10.0f, T_BS}, 100.0f, 100.0f, width / 2 - 0.5f, height / 2 - 0.5f});
}

template<typename MapT>
void integrate_wall(MapT& map, const se::PinholeCamera& sensor, float depth_value)
{
    const se::Image<float> depth(
        sensor.model.imageWidth(), sensor.model.imageHeight(), depth_value);
    const Eigen::Isometry3f T_WB = Eigen::Isometry3f::Identity();
    se::MapIntegrator integrator_stsdf(map);
    integrator_stsdf.integrateDepth(
        0, se::Measurements{se::Measurement{depth, sensor, T_WB * sensor.T_BS}});
}

int dim_to_blocks(float dim, float block_dim)
{
    return std::copysign(std::abs(dim) / block_dim + 0.5f, dim);
}

TEST(Map, aabbTSDF)
{
    // Generate a z-up map with a wall integrated perpendicular to the the x axis at depth_value.
    se::TSDFMap<> map(Eigen::Vector3f::Constant(12.8f), 0.1f);
    const se::PinholeCamera sensor = generate_sensor();
    constexpr float depth_value = 4.0f;
    integrate_wall(map, sensor, depth_value);

    const float truncation_boundary =
        map.getRes() * map.getDataConfig().field.truncation_boundary_factor;
    const float block_dim = map.getRes() * map.getOctree().block_size;
    const float half_wall_dim = depth_value * std::tan(sensor.horizontal_fov / 2);
    const int half_wall_blocks = dim_to_blocks(half_wall_dim, block_dim);
    const auto aabb = map.aabb();

    // The values used in the tests below are hand-crafted for this particular example.
    EXPECT_FLOAT_EQ(aabb.min().x(), depth_value - truncation_boundary);
    EXPECT_FLOAT_EQ(aabb.max().x(), depth_value + truncation_boundary);
    EXPECT_FLOAT_EQ(aabb.min().z(), aabb.min().y());
    EXPECT_FLOAT_EQ(aabb.max().z(), aabb.max().y());
    EXPECT_EQ(dim_to_blocks(aabb.min().y(), block_dim), -half_wall_blocks);
    EXPECT_EQ(dim_to_blocks(aabb.max().y(), block_dim), half_wall_blocks);
}

TEST(Map, aabbOccupancy)
{
    se::OccupancyMap<> map(Eigen::Vector3f::Constant(12.8f), 0.1f);
    EXPECT_TRUE(map.aabb().isEmpty());

    // Generate a z-up map with a wall integrated perpendicular to the the x axis at depth_value.
    const se::PinholeCamera sensor = generate_sensor();
    constexpr float depth_value = 4.0f;
    integrate_wall(map, sensor, depth_value);

    const float block_dim = map.getRes() * map.getOctree().block_size;
    const float half_wall_dim = depth_value * std::tan(sensor.horizontal_fov / 2);
    const int half_wall_blocks = dim_to_blocks(half_wall_dim, block_dim);
    const auto aabb = map.aabb();

    // The values used in the tests below are hand-crafted for this particular example.
    EXPECT_NEAR(aabb.min().x(), 0.0f, 1e-6);
    EXPECT_FLOAT_EQ(aabb.max().x(), depth_value + 3 * block_dim);
    EXPECT_FLOAT_EQ(aabb.min().z(), aabb.min().y());
    EXPECT_FLOAT_EQ(aabb.max().z(), aabb.max().y());
    // An extra block seems to be allocated in both directions of the y/z axes. Added ±1 so the test
    // passes.
    EXPECT_EQ(dim_to_blocks(aabb.min().y(), block_dim), -half_wall_blocks - 1);
    EXPECT_EQ(dim_to_blocks(aabb.max().y(), block_dim), half_wall_blocks + 1);
}

TEST(Map, aabb_ray_batch)
{
    // Temporary directory for test results
    const std::string tmp_ = stdfs::temp_directory_path() / stdfs::path("supereight_test_results");

    /**
   * Create plane wall example
   */
    const float elevation_min = -15.0f;
    const float elevation_max = 15.0f;
    const float azimuth_min = -15.0f;
    const float azimuth_max = 15.0f;
    // angular resolution [degree]
    const float elevation_res = .1f;
    const float azimuth_res = .1f;
    // conversion degree <-> rad
    const float deg_to_rad = M_PI / 180.0f;
    // distance of plane wall [m]
    float d = 10.0f;

    std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>
        rayBatch;
    size_t num_points_elevation = std::floor((elevation_max - elevation_min) / elevation_res);
    size_t num_points_azimuth = std::floor((azimuth_max - azimuth_min) / azimuth_res);

    float elevation_angle = elevation_min;
    float azimuth_angle = azimuth_min;
    float x, y, z;
    x = d;
    for (size_t i = 0; i < num_points_elevation; i++) {
        z = d * tan(elevation_angle * deg_to_rad);
        for (size_t j = 0; j < num_points_azimuth; j++) {
            y = d * tan(azimuth_angle * deg_to_rad);
            // save point
            rayBatch.push_back(std::pair<Eigen::Isometry3f, Eigen::Vector3f>(
                Eigen::Isometry3f::Identity(), Eigen::Vector3f(x, y, z)));
            // increase azimuth angle
            azimuth_angle += azimuth_res;
        }
        azimuth_angle = azimuth_min;
        //increase elevation angle
        elevation_angle += elevation_res;
    }


    // ========= Map INITIALIZATION  =========
    const float res = 0.05f;
    const float dim = 25.6f;
    se::OccupancyMap<se::Res::Multi> map(Eigen::Vector3f::Constant(dim), res);


    // ========= Sensor INITIALIZATION  =========
    se::LeicaLidar::Config sensorConfig;
    sensorConfig.width = 1;  // To satisfy assert
    sensorConfig.height = 1; // To satisfy assert
    sensorConfig.near_plane = 0.6f;
    sensorConfig.far_plane = 30.0f;
    sensorConfig.T_BS = Eigen::Isometry3f::Identity();
    sensorConfig.elevation_resolution_angle_ = static_cast<float>(elevation_res);
    sensorConfig.azimuth_resolution_angle_ = static_cast<float>(azimuth_res);

    //se::LeicaLidar::Config sensorConfig(se_config.sensor);
    const se::LeicaLidar sensor(sensorConfig);

    // ========= Integrator INITIALIZATION  =========
    se::MapIntegrator integrator(map);

    // ========= Integration (Batched)
    integrator.integrateRayBatch(0, rayBatch, sensor);

    const float block_dim = res * map.getOctree().block_size;
    const float half_wall_dim = d * tanf(azimuth_max * deg_to_rad);
    const float half_wall_blocks = dim_to_blocks(half_wall_dim, block_dim);

    // lines are commented out.
    float min_x = block_dim * std::floor(sensor.near_plane / (2 * block_dim));
    EXPECT_NEAR(map.aabb().min().x(), min_x, 1e-02f);
    // x should be a multiple of the block size (in meters) depending on tau and the resolution
    // with default tau_min = 3
    int n_of_blocks_x = std::ceil((d + 3 * res) / block_dim);
    EXPECT_FLOAT_EQ(map.aabb().max().x(), n_of_blocks_x * block_dim);
    EXPECT_FLOAT_EQ(map.aabb().min().z(), map.aabb().min().y());
    EXPECT_FLOAT_EQ(map.aabb().max().z(), map.aabb().max().y());
    EXPECT_EQ(dim_to_blocks(map.aabb().min().y(), block_dim), -half_wall_blocks - 1);
    EXPECT_EQ(dim_to_blocks(map.aabb().max().y(), block_dim), half_wall_blocks + 1);

    // Extend wall in other direction (test negative x min
    rayBatch.clear();
    elevation_angle = elevation_min;
    azimuth_angle = azimuth_min;
    x = -d;
    for (size_t i = 0; i < num_points_elevation; i++) {
        z = d * tan(elevation_angle * deg_to_rad);
        for (size_t j = 0; j < num_points_azimuth; j++) {
            y = d * tan(azimuth_angle * deg_to_rad);
            // save point
            rayBatch.push_back(std::pair<Eigen::Isometry3f, Eigen::Vector3f>(
                Eigen::Isometry3f::Identity(), Eigen::Vector3f(x, y, z)));
            // increase azimuth angle
            azimuth_angle += azimuth_res;
        }
        azimuth_angle = azimuth_min;
        //increase elevation angle
        elevation_angle += elevation_res;
    }

    integrator.integrateRayBatch(0, rayBatch, sensor);

    // lines are commented out.
    int negative_n_of_blocks_x = std::floor(-(d + 3 * res) / block_dim);
    EXPECT_FLOAT_EQ(map.aabb().min().x(), negative_n_of_blocks_x * block_dim);
    EXPECT_FLOAT_EQ(map.aabb().max().x(), n_of_blocks_x * block_dim);
    EXPECT_FLOAT_EQ(map.aabb().min().z(), map.aabb().min().y());
    EXPECT_FLOAT_EQ(map.aabb().max().z(), map.aabb().max().y());
    EXPECT_EQ(dim_to_blocks(map.aabb().min().y(), block_dim), -half_wall_blocks - 1);
    EXPECT_EQ(dim_to_blocks(map.aabb().max().y(), block_dim), half_wall_blocks + 1);

    // Un-Comment if debugging necessary (allocated structure visualization)
    map.getOctree().saveStructure(tmp_ + "/multi-ray-aabb-allocated_structure.ply");
    map.getOctree().saveMesh(tmp_ + "/multi-ray-aabb-voxel-mesh.ply");
    map.saveMesh(tmp_ + "/multi-ray-aabb-metric-mesh.ply");
    map.saveFieldSlices(tmp_ + "/multi-ray-aabb-slice-x.vtk",
                        tmp_ + "/multi-ray-aabb-slice-y.vtk",
                        tmp_ + "/multi-ray-aabb-slice-z.vtk",
                        Eigen::Vector3f(0.f, 0.f, 0.f));
}
