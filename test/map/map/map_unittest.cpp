/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/integrator/map_integrator.hpp>

TEST(Map, Gradient)
{
    // Distance at which surface (plane wall) for test case will be created
    constexpr float surface_distance = 15;
    // Map config
    const Eigen::Vector3f map_dim = Eigen::Vector3f::Constant(32);
    constexpr float map_res = 0.03;
    // Sensor config
    se::PinholeCamera::Config sensor_config;
    sensor_config.width = 640;
    sensor_config.height = 480;
    sensor_config.fx = 525;
    sensor_config.fy = sensor_config.fx;
    sensor_config.cx = (sensor_config.width - 1) / 2.0f;
    sensor_config.cy = (sensor_config.height - 1) / 2.0f;
    sensor_config.near_plane = 0.4f;
    sensor_config.far_plane = 20.0f;
    // Data Config
    se::OccupancyData::Config data_config;
    data_config.field.tau_min_factor = 20;
    data_config.field.tau_max_factor = 20;
    data_config.field.k_tau = 1;
    data_config.field.sigma_min_factor = 1;
    data_config.field.sigma_max_factor = 3;
    data_config.field.k_sigma = 0.05f;

    // Create Map
    se::OccupancyMap<se::Res::Multi> map(map_dim, map_res, data_config);
    // Create a pinhole camera and uniform depth image
    const se::PinholeCamera sensor(sensor_config, 2);
    const se::Image<float> depth_img(
        sensor.model.imageWidth(), sensor.model.imageHeight(), surface_distance);
    // Integrate depth image from an identity T_WS.
    se::MapIntegrator integrator(map);
    integrator.integrateDepth(
        0, se::Measurements{se::Measurement{depth_img, sensor, Eigen::Isometry3f::Identity()}});

    // Test that points in free space have 0 gradients
    const std::array free_points{
        Eigen::Vector3f(0, 0, 4), Eigen::Vector3f(0, 0, 6), Eigen::Vector3f(0, 0, 8)};
    for (const auto& point : free_points) {
        // Check free space occupancy value
        const std::optional<se::field_t> occupancy = map.getFieldInterp(point);
        ASSERT_TRUE(occupancy);
        EXPECT_FLOAT_EQ(*occupancy, data_config.field.log_odd_min);
        // Check for zero gradients
        const std::optional<Eigen::Vector3f> gradient = map.getFieldGrad(point);
        ASSERT_TRUE(gradient);
        EXPECT_FLOAT_EQ(gradient->norm(), 0.0f);
    }

    // Test the derivatives of points in the transition between free and occupied space. This is the
    // linear occupancy increase region of the inverse sensor model. Compare with a simple numerical
    // gradient.
    const std::array linear_points{Eigen::Vector3f(0, 0, 14.85f),
                                   Eigen::Vector3f(0, 0, 14.90f),
                                   Eigen::Vector3f(0, 0, 14.95f),
                                   Eigen::Vector3f(0, 0, 15.00f)};
    for (const auto& point : linear_points) {
        // Check if gradient matches numeric differences gradient
        const float delta = map_res / 10;
        bool gradient_numeric_valid = true;
        Eigen::Vector3f gradient_numeric = Eigen::Vector3f::Zero();

        // Numeric field gradient using the central difference quotient
        for (int i = 0; i < gradient_numeric.size(); i++) {
            const Eigen::Vector3f dir = Eigen::Vector3f::Unit(i);
            const std::optional<se::field_t> occupancy_m = map.getFieldInterp(point - delta * dir);
            const std::optional<se::field_t> occupancy_p = map.getFieldInterp(point + delta * dir);
            if (!occupancy_p || !occupancy_m) {
                gradient_numeric_valid = false;
                break;
            }
            if (std::fabs(*occupancy_p - *occupancy_m) > 1e-06) {
                gradient_numeric[i] = (*occupancy_p - *occupancy_m) / (2 * delta);
            }
        }
        ASSERT_TRUE(gradient_numeric_valid);

        // Access Field Gradient
        const std::optional<Eigen::Vector3f> gradient = map.getFieldGrad(point);
        ASSERT_TRUE(gradient);

        // The error between the supereight and numerical gradients can be rather big due to
        // different space discretization and gradient computation methods.
        EXPECT_NEAR(gradient_numeric.x(), gradient->x(), 1e-02);
        EXPECT_NEAR(gradient_numeric.y(), gradient->y(), 1e-02);
        EXPECT_NEAR(gradient_numeric.z(), gradient->z(), 1e-02);
    }

    // Test that points in occupied space have 0 gradients
    const std::array occupied_points{Eigen::Vector3f(0, 0, 15.35f)};
    for (const auto& point : occupied_points) {
        // Check free space occupancy value
        const std::optional<se::field_t> occupancy = map.getFieldInterp(point);
        ASSERT_TRUE(occupancy);
        EXPECT_GT(*occupancy, 0.0f);
        // Check for zero gradients
        const std::optional<Eigen::Vector3f> gradient = map.getFieldGrad(point);
        ASSERT_TRUE(gradient);
        EXPECT_FLOAT_EQ(gradient->norm(), 0.0f);
    }
}



// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale)
{
    Eigen::Vector3i adapted_coord;
    adapted_coord.x() = (coord.x() >> scale) << scale;
    adapted_coord.y() = (coord.y() >> scale) << scale;
    adapted_coord.z() = (coord.z() >> scale) << scale;
    return adapted_coord;
}

TEST(Map, Interpolation)
{
    const Eigen::Vector3f map_dim(32.f, 32.f, 32.f);
    const float map_res(1.f);
    se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res);

    Eigen::Vector3i block_coord;
    Eigen::Vector3i coord_ought;
    Eigen::Vector3i coord_is;

    typedef typename se::TSDFMap<se::Res::Single>::DataType DataType;
    typedef typename se::TSDFMap<se::Res::Single>::OctreeType OctreeType;
    typedef typename se::TSDFMap<se::Res::Single>::BlockType BlockType;

    int block_size = BlockType::size;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> block_coords = {
        Eigen::Vector3i(0, 0, 0),
        Eigen::Vector3i(block_size, 0, 0),
        Eigen::Vector3i(0, block_size, 0),
        Eigen::Vector3i(block_size, block_size, 0),
        Eigen::Vector3i(0, 0, block_size),
        Eigen::Vector3i(block_size, 0, block_size),
        Eigen::Vector3i(0, block_size, block_size),
        Eigen::Vector3i(block_size, block_size, block_size)};

    OctreeType& octree = map_tsdf.getOctree();

    BlockType* block_ptr = nullptr;

    for (size_t i = 0; i < block_coords.size(); i++) {
        const Eigen::Vector3i block_coord = block_coords[i];
        coord_ought = adapt_to_scale(block_coord, octree.max_block_scale);
        se::key_t voxel_key;
        se::keyops::encode_key(block_coord, 0, voxel_key);
        block_ptr =
            static_cast<BlockType*>(se::allocator::block(voxel_key, octree, octree.getRoot()));
        coord_is = block_ptr->coord;
        EXPECT_EQ(coord_ought, coord_is);
        for (size_t voxel_idx = 0; voxel_idx < block_ptr->size_cu; voxel_idx++) {
            DataType data;
            data.field.tsdf = i;
            data.field.weight = 1;
            block_ptr->data(voxel_idx) = data;
        }
    }

    auto interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(+2, +2, +2));
    EXPECT_FALSE(interp_field_value);
}

TEST(Map, initialization)
{
    struct TestData {
        float dim;
        float res;
        int size;
    };

    const std::vector<TestData> data{
        {12.8f, 0.1f, 128},
        {25.6f, 0.1f, 256},
        {51.2f, 0.1f, 512},
        {10.24f, 0.01f, 1024},
        {20.48f, 0.01f, 2048},
        {10.24f, 0.02f, 512},
        {20.48f, 0.02f, 1024},
        {10.0f, 0.05f, 256},
        {100.0f, 0.05f, 2048},
    };

    for (const auto& d : data) {
        se::TSDFMap<se::Res::Single> map(Eigen::Vector3f::Constant(d.dim), d.res);
        EXPECT_GE(map.getDim().x(), d.dim);
        EXPECT_FLOAT_EQ(map.getRes(), d.res);
        EXPECT_EQ(map.getOctree().getSize(), d.size);
    }
}
