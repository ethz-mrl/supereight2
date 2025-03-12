/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/sensor/sensor.hpp"

void se::PinholeCamera::Config::readYaml(const std::string& filename)
{
    // Read the base class members.
    SensorBase<PinholeCamera>::Config::readYaml(filename);

    // Open the file for reading.
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];

    // Read the config parameters.
    se::yaml::subnode_as_float(node, "fx", fx);
    se::yaml::subnode_as_float(node, "fy", fy);
    se::yaml::subnode_as_float(node, "cx", cx);
    se::yaml::subnode_as_float(node, "cy", cy);
}

se::PinholeCamera::Config
se::PinholeCamera::Config::operator/(const float downsampling_factor) const
{
    return se::PinholeCamera::Config{
        se::SensorBase<se::PinholeCamera>::Config::operator/(downsampling_factor),
        fx / downsampling_factor,
        fy / downsampling_factor,
        (cx + 0.5f) / downsampling_factor - 0.5f,
        (cy + 0.5f) / downsampling_factor - 0.5f,
    };
}



std::ostream& se::operator<<(std::ostream& os, const se::PinholeCamera::Config& c)
{
    os << static_cast<const SensorBase<PinholeCamera>::Config&>(c);
    os << str_utils::value_to_pretty_str(c.fx, "fx") << " px\n";
    os << str_utils::value_to_pretty_str(c.fy, "fy") << " px\n";
    os << str_utils::value_to_pretty_str(c.cx, "cx") << " px\n";
    os << str_utils::value_to_pretty_str(c.cy, "cy") << " px\n";
    return os;
}



// Explicit template class instantiation
template class srl::projection::PinholeCamera<srl::projection::NoDistortion>;

// Used for initializing a PinholeCamera.
const srl::projection::NoDistortion _distortion;



se::PinholeCamera::PinholeCamera(const Config& c) :
        se::SensorBase<se::PinholeCamera>(c),
        model(c.width, c.height, c.fx, c.fy, c.cx, c.cy, _distortion),
        scaled_pixel(1 / c.fx)
{
    left_hand_frame = (c.fx < 0.0f) ^ (c.fy < 0.0f);

    computeFrustumVertices();
    computeFrustumNormals();

    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane > 0.0f);
    assert(c.far_plane > c.near_plane);
    assert(!std::isnan(c.fx));
    assert(!std::isnan(c.fy));
    assert(!std::isnan(c.cx));
    assert(!std::isnan(c.cy));

    horizontal_fov = 2.0f * atanf(c.width / (2.0f * c.fx));
    vertical_fov = 2.0f * atanf(c.height / (2.0f * c.fy));
}



se::PinholeCamera::PinholeCamera(const Config& c, const float dsf) :
        se::SensorBase<se::PinholeCamera>(c),
        model(c.width / dsf,
              c.height / dsf,
              c.fx / dsf,
              c.fy / dsf,
              (c.cx + 0.5f) / dsf - 0.5f,
              (c.cy + 0.5f) / dsf - 0.5f,
              _distortion),
        scaled_pixel(1 / (c.fx / dsf))
{
    left_hand_frame = (c.fx < 0.0f) ^ (c.fy < 0.0f);

    computeFrustumVertices();
    computeFrustumNormals();

    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane >= 0.f);
    assert(c.far_plane > c.near_plane);
    assert(!std::isnan(c.fx));
    assert(!std::isnan(c.fy));
    assert(!std::isnan(c.cx));
    assert(!std::isnan(c.cy));

    horizontal_fov = 2.0f * atanf(c.width / (2.0f * c.fx));
    vertical_fov = 2.0f * atanf(c.height / (2.0f * c.fy));
}



se::PinholeCamera::PinholeCamera(const PinholeCamera& pc, const float dsf) :
        se::SensorBase<se::PinholeCamera>(pc),
        model(pc.model.imageWidth() / dsf,
              pc.model.imageHeight() / dsf,
              pc.model.focalLengthU() / dsf,
              pc.model.focalLengthV() / dsf,
              ((pc.model.imageCenterU() + 0.5f) / dsf - 0.5f),
              ((pc.model.imageCenterV() + 0.5f) / dsf - 0.5f),
              _distortion),
        scaled_pixel(1 / (pc.model.focalLengthU() / dsf))
{
    computeFrustumVertices();
    computeFrustumNormals();

    horizontal_fov = 2.0f * atanf(pc.model.imageWidth() / (2.0f * pc.model.focalLengthU()));
    vertical_fov = 2.0f * atanf(pc.model.imageHeight() / (2.0f * pc.model.focalLengthV()));
}



int se::PinholeCamera::computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre_S,
                                                   const float map_res,
                                                   const int last_scale,
                                                   const int min_scale,
                                                   const int max_block_scale) const
{
    const float dist = block_centre_S.z();
    // Compute the side length in metres of a pixel projected dist metres from the camera
    const float pixel_dim = dist * scaled_pixel;
    const float pv_ratio = pixel_dim / map_res;
    int scale = 0;
    for (const float scale_ratio : pixel_voxel_ratio_per_scale) {
        if (pv_ratio < scale_ratio) {
            break;
        }
        scale++;
    }
    scale = std::min(scale, max_block_scale);

    Eigen::Vector3f block_centre_S_hyst = block_centre_S;
    bool recompute = false;
    if (scale > last_scale && min_scale != -1) {
        block_centre_S_hyst.z() -= 0.25;
        recompute = true;
    }
    else if (scale < last_scale && min_scale != -1) {
        block_centre_S_hyst.z() += 0.25;
        recompute = true;
    }

    if (recompute) {
        return computeIntegrationScaleImpl(
            block_centre_S_hyst, map_res, last_scale, -1, max_block_scale);
    }
    else {
        return scale;
    }
}



bool se::PinholeCamera::pointInFrustumImpl(const Eigen::Vector3f& point_S) const
{
    for (int i = 0; i < FrustumNormal::Num; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = point_S.homogeneous().dot(frustum_normals_S.col(i));
        if (distance < 0.0f) {
            // A negative distance means that the point is located on the opposite
            // halfspace than the one the plane normal is pointing towards
            return false;
        }
    }
    return true;
}



bool se::PinholeCamera::pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const
{
    // Skip the far plane normal
    for (int i = 0; i < FrustumNormal::Far; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = point_S.homogeneous().dot(frustum_normals_S.col(i));
        if (distance < 0.0f) {
            // A negative distance means that the point is located on the opposite
            // halfspace than the one the plane normal is pointing towards
            return false;
        }
    }
    return true;
}



bool se::PinholeCamera::sphereInFrustumImpl(const Eigen::Vector3f& center_S,
                                            const float radius) const
{
    for (int i = 0; i < FrustumNormal::Num; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = center_S.homogeneous().dot(frustum_normals_S.col(i));
        if (distance < -radius) {
            // Instead of testing for negative distance as in
            // se::PinholeCamera::pointInFrustum, test for distance smaller than
            // -radius so that the test is essentially performed on the plane offset
            // by radius.
            return false;
        }
    }
    return true;
}



bool se::PinholeCamera::sphereInFrustumInfImpl(const Eigen::Vector3f& center_S,
                                               const float radius) const
{
    // Skip the far plane normal
    for (int i = 0; i < FrustumNormal::Far; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = center_S.homogeneous().dot(frustum_normals_S.col(i));
        if (distance < -radius) {
            // Instead of testing for negative distance as in
            // se::PinholeCamera::pointInFrustum, test for distance smaller than
            // -radius so that the test is essentially performed on the plane offset
            // by radius.
            return false;
        }
    }
    return true;
}



se::PinholeCamera se::PinholeCamera::testInstance()
{
    constexpr int width = 640;
    constexpr int height = 480;
    // The focal length was scientifically eyeballed so that there's minimal overlap between frusta
    // when integrating 4 uniform depth images at 5 metres from 4 poses at 0°, 90°, 180° and 270°
    // yaw in a map with a 1 cm resolution.
    constexpr float f = 315.0f;
    constexpr float cx = width / 2 - 0.5f;
    constexpr float cy = height / 2 - 0.5f;
    // Transforms from the z-forward, x-right frame S to the x-forward, z-up frame B.
    const Eigen::Isometry3f T_BS(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY())
                                 * Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    return se::PinholeCamera({{width, height, 0.1f, 10.0f, T_BS}, f, f, cx, cy});
}



void se::PinholeCamera::computeFrustumVertices()
{
    Eigen::Vector3f point_S;
    // Back-project the frame corners to get the frustum vertices
    // Top left
    model.backProject(Eigen::Vector2f(0.0f, 0.0f), &point_S);
    frustum_vertices_S.col(FrustumVertex::TopLeftNear) = near_plane * point_S;
    frustum_vertices_S.col(FrustumVertex::TopLeftFar) = far_plane * point_S;
    // Top right
    model.backProject(Eigen::Vector2f(model.imageWidth(), 0.0f), &point_S);
    frustum_vertices_S.col(FrustumVertex::TopRightNear) = near_plane * point_S;
    frustum_vertices_S.col(FrustumVertex::TopRightFar) = far_plane * point_S;
    // Bottom right
    model.backProject(Eigen::Vector2f(model.imageWidth(), model.imageHeight()), &point_S);
    frustum_vertices_S.col(FrustumVertex::BottomRightNear) = near_plane * point_S;
    frustum_vertices_S.col(FrustumVertex::BottomRightFar) = far_plane * point_S;
    // Bottom left
    model.backProject(Eigen::Vector2f(0.0f, model.imageHeight()), &point_S);
    frustum_vertices_S.col(FrustumVertex::BottomLeftNear) = near_plane * point_S;
    frustum_vertices_S.col(FrustumVertex::BottomLeftFar) = far_plane * point_S;
}



void se::PinholeCamera::computeFrustumNormals()
{
    // The w vector component corresponds to the distance of the plane from the
    // origin. It should be 0 for all planes other than the near and far planes.
    // Left plane vector.
    frustum_normals_S.col(FrustumNormal::Left).head<3>() =
        se::math::plane_normal(frustum_vertices_S.col(FrustumVertex::TopLeftFar),
                               frustum_vertices_S.col(FrustumVertex::TopLeftNear),
                               frustum_vertices_S.col(FrustumVertex::BottomLeftNear));
    frustum_normals_S.col(FrustumNormal::Left).w() = 0.0f;
    // Right plane vector.
    frustum_normals_S.col(FrustumNormal::Right).head<3>() =
        se::math::plane_normal(frustum_vertices_S.col(FrustumVertex::TopRightNear),
                               frustum_vertices_S.col(FrustumVertex::TopRightFar),
                               frustum_vertices_S.col(FrustumVertex::BottomRightFar));
    frustum_normals_S.col(FrustumNormal::Right).w() = 0.0f;
    // Bottom plane vector.
    frustum_normals_S.col(FrustumNormal::Bottom).head<3>() =
        se::math::plane_normal(frustum_vertices_S.col(FrustumVertex::BottomLeftFar),
                               frustum_vertices_S.col(FrustumVertex::BottomLeftNear),
                               frustum_vertices_S.col(FrustumVertex::BottomRightNear));
    frustum_normals_S.col(FrustumNormal::Bottom).w() = 0.0f;
    // Top plane vector.
    frustum_normals_S.col(FrustumNormal::Top).head<3>() =
        se::math::plane_normal(frustum_vertices_S.col(FrustumVertex::TopRightFar),
                               frustum_vertices_S.col(FrustumVertex::TopRightNear),
                               frustum_vertices_S.col(FrustumVertex::TopLeftNear));
    frustum_normals_S.col(FrustumNormal::Top).w() = 0.0f;
    // Near plane vector.
    frustum_normals_S.col(FrustumNormal::Near) = Eigen::Vector4f(0.f, 0.f, 1.f, -near_plane);
    // Far plane vector.
    frustum_normals_S.col(FrustumNormal::Far) = Eigen::Vector4f(0.f, 0.f, -1.f, far_plane);
    // The directions of all normal vectors must be multiplied with sign(fx * fy) to account for
    // negative focal lengths. The near and far plane normals are already set with the correct
    // direction.
    float sign = model.focalLengthU() * model.focalLengthV();
    sign = sign / fabsf(sign);
    for (int i = 0; i < FrustumNormal::Near; i++) {
        frustum_normals_S.col(i).head<3>() *= sign;
    }
}
