/*
 * SPDX-FileCopyrightText: 2023-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/sensor/sensor.hpp>

static se::PinholeCamera::Config
pinhole_config(const float near_plane = se::PinholeCamera::Config().near_plane,
               const float far_plane = se::PinholeCamera::Config().far_plane)
{
    // The other se::PinholeCamera::Config members are default-initialized.
    return se::PinholeCamera::Config{
        {640, 480, near_plane, far_plane}, 525.0f, 525.0f, 319.5f, 239.5f};
}



TEST(PinholeCamera, downsampling)
{
    const auto config = pinhole_config();

    const se::PinholeCamera c0(config, 2.0f);
    EXPECT_EQ(c0.model.imageWidth(), 320);
    EXPECT_EQ(c0.model.imageHeight(), 240);
    EXPECT_FLOAT_EQ(c0.model.focalLengthU(), 262.5f);
    EXPECT_FLOAT_EQ(c0.model.focalLengthV(), 262.5f);
    EXPECT_FLOAT_EQ(c0.model.imageCenterU(), 159.5f);
    EXPECT_FLOAT_EQ(c0.model.imageCenterV(), 119.5f);
    EXPECT_FLOAT_EQ(c0.horizontal_fov, 1.09478578f);
    EXPECT_FLOAT_EQ(c0.vertical_fov, 0.85755605f);

    const se::PinholeCamera c1(c0, 2.0f);
    EXPECT_EQ(c1.model.imageWidth(), 160);
    EXPECT_EQ(c1.model.imageHeight(), 120);
    EXPECT_FLOAT_EQ(c1.model.focalLengthU(), 131.25f);
    EXPECT_FLOAT_EQ(c1.model.focalLengthV(), 131.25f);
    EXPECT_FLOAT_EQ(c1.model.imageCenterU(), 79.5f);
    EXPECT_FLOAT_EQ(c1.model.imageCenterV(), 59.5f);
    EXPECT_FLOAT_EQ(c1.horizontal_fov, 1.09478578f);
    EXPECT_FLOAT_EQ(c1.vertical_fov, 0.85755605f);

    const se::PinholeCamera c2(config, 4.0f);
    EXPECT_EQ(c1.model.imageWidth(), c2.model.imageWidth());
    EXPECT_EQ(c1.model.imageHeight(), c2.model.imageHeight());
    EXPECT_FLOAT_EQ(c1.model.focalLengthU(), c2.model.focalLengthU());
    EXPECT_FLOAT_EQ(c1.model.focalLengthV(), c2.model.focalLengthV());
    EXPECT_FLOAT_EQ(c1.model.imageCenterU(), c2.model.imageCenterU());
    EXPECT_FLOAT_EQ(c1.model.imageCenterV(), c2.model.imageCenterV());
    EXPECT_FLOAT_EQ(c1.horizontal_fov, c2.horizontal_fov);
    EXPECT_FLOAT_EQ(c1.vertical_fov, c2.vertical_fov);
}



// Test the PinholeCamera::*InFrustum*() functions for various values of the near and far planes.
struct Params {
    float near;
    float far;
};

class PinholeCamera : public ::testing::TestWithParam<Params> {
    protected:
    const se::PinholeCamera::Config config_ = pinhole_config(GetParam().near, GetParam().far);
};

INSTANTIATE_TEST_SUITE_P(NearFar,
                         PinholeCamera,
                         ::testing::Values(Params{se::PinholeCamera::Config().near_plane,
                                                  se::PinholeCamera::Config().far_plane},
                                           Params{0.0f, 1000.0f}));

TEST_P(PinholeCamera, inFrustum)
{
    const se::PinholeCamera c(config_);

    using FV = se::PinholeCamera::FrustumVertex;
    const auto& fv = c.frustum_vertices_S;
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::TopLeftNear)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::TopRightNear)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::BottomRightNear)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::BottomLeftNear)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::TopLeftFar)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::TopRightFar)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::BottomRightFar)));
    EXPECT_TRUE(c.pointInFrustum(fv.col(FV::BottomLeftFar)));

    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::TopLeftNear), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::TopRightNear), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::BottomRightNear), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::BottomLeftNear), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::TopLeftFar), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::TopRightFar), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::BottomRightFar), 0.0f));
    EXPECT_TRUE(c.sphereInFrustum(fv.col(FV::BottomLeftFar), 0.0f));

    const Eigen::Vector3f point_S_0(0.0f, 0.0f, config_.near_plane);
    EXPECT_TRUE(c.pointInFrustum(point_S_0));
    EXPECT_TRUE(c.pointInFrustumInf(point_S_0));
    EXPECT_TRUE(c.sphereInFrustum(point_S_0, 0.01f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_0, 0.0f));

    const Eigen::Vector3f point_S_1(-0.04f, 0.08f, config_.far_plane);
    EXPECT_TRUE(c.pointInFrustum(point_S_1));
    EXPECT_TRUE(c.pointInFrustumInf(point_S_1));
    EXPECT_TRUE(c.sphereInFrustum(point_S_1, 0.0f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_1, 0.5f));

    const Eigen::Vector3f point_S_2(0.0f, 0.0f, config_.near_plane - 0.01f);
    EXPECT_FALSE(c.pointInFrustum(point_S_2));
    EXPECT_FALSE(c.pointInFrustumInf(point_S_2));
    EXPECT_FALSE(c.sphereInFrustum(point_S_2, 0.0f));
    EXPECT_FALSE(c.sphereInFrustumInf(point_S_2, 0.0f));
    EXPECT_FALSE(c.sphereInFrustum(point_S_2, 0.001f));
    EXPECT_FALSE(c.sphereInFrustumInf(point_S_2, 0.001f));
    EXPECT_TRUE(c.sphereInFrustum(point_S_2, 0.01f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_2, 0.01f));

    const Eigen::Vector3f point_S_3(-0.04f, 0.08f, config_.far_plane + 0.5f);
    EXPECT_FALSE(c.pointInFrustum(point_S_3));
    EXPECT_TRUE(c.pointInFrustumInf(point_S_3));
    EXPECT_FALSE(c.sphereInFrustum(point_S_3, 0.0f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_3, 0.0f));
    EXPECT_FALSE(c.sphereInFrustum(point_S_3, 0.1f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_3, 0.1f));
    EXPECT_TRUE(c.sphereInFrustum(point_S_3, 0.5f));
    EXPECT_TRUE(c.sphereInFrustumInf(point_S_3, 0.5f));

    const Eigen::Vector3f point_S_4(1000.0f, 0.0f, (config_.near_plane + config_.far_plane) / 2.0f);
    EXPECT_FALSE(c.pointInFrustum(point_S_4));
    EXPECT_FALSE(c.pointInFrustumInf(point_S_4));
    EXPECT_FALSE(c.sphereInFrustum(point_S_4, 0.0f));
    EXPECT_FALSE(c.sphereInFrustumInf(point_S_4, 0.0f));
    EXPECT_FALSE(c.sphereInFrustum(point_S_4, 1.0f));
    EXPECT_FALSE(c.sphereInFrustumInf(point_S_4, 10.0f));
}
