/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/sensor/sensor.hpp"

void se::Lidar::Config::readYaml(const std::string& filename)
{
    // Read the base class members.
    SensorBase<Lidar>::Config::readYaml(filename);

    // Open the file for reading.
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];

    se::yaml::subnode_as_float(node, "elevation_resolution_angle", elevation_resolution_angle_);
    se::yaml::subnode_as_float(node, "azimuth_resolution_angle", azimuth_resolution_angle_);
}



std::ostream& se::operator<<(std::ostream& os, const se::Lidar::Config& c)
{
    os << static_cast<const SensorBase<Lidar>::Config&>(c);
    os << str_utils::value_to_pretty_str(c.elevation_resolution_angle_,
                                         "elevation_resolution_angle")
       << " degrees\n";
    os << str_utils::value_to_pretty_str(c.azimuth_resolution_angle_, "azimuth_resolution_angle")
       << " degrees\n";
    return os;
}



se::Lidar::Lidar(const Config& c) :
        se::SensorBase<se::Lidar>(c),
        azimuth_resolution_angle(c.azimuth_resolution_angle_),
        elevation_resolution_angle(c.elevation_resolution_angle_)
{
    assert(c.near_plane > 0.f);
    assert(c.far_plane > c.near_plane);

    max_ray_angle = std::max(c.azimuth_resolution_angle_, c.elevation_resolution_angle_);
    horizontal_fov = 2.0f * M_PI;

    constexpr float deg_to_rad = M_PI / 180.0f;

    const float min_elevation = -90.0f;
    min_elevation_rad = min_elevation * deg_to_rad;
    const float max_elevation = 90.0f;
    max_elevation_rad = max_elevation * deg_to_rad;
    vertical_fov =
        deg_to_rad * (max_elevation - min_elevation); // should be 180 degree respectively PI

    pixel_dim_tan = 2.0f * std::tan(max_ray_angle * 0.5f * deg_to_rad);
}




int se::Lidar::blockIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
                                              const float map_res,
                                              const int last_scale,
                                              const int min_scale,
                                              const int max_block_scale) const
{
    //constexpr float deg_to_rad = M_PI / 180.0f;
    const float dist = block_centre.norm();
    // Compute the side length in metres of a pixel projected dist metres from
    // the camera. This computes the chord length corresponding to the ray angle
    // at distance dist.
    const float pixel_dim = dist * pixel_dim_tan;
    // Compute the ratio using the worst case voxel_dim (space diagonal)
    const float pv_ratio_denominator = 1.0f / (std::sqrt(3.0f) * map_res);
    const float pv_ratio = pixel_dim * pv_ratio_denominator;
    int scale = 0;
    for (const float scale_ratio : pixel_voxel_ratio_per_scale) {
        if (pv_ratio < scale_ratio) {
            break;
        }
        scale++;
    }
    scale = std::min(scale, max_block_scale);
    Eigen::Vector3f block_centre_hyst = block_centre;
    bool recompute = false;
    if (scale > last_scale && min_scale != -1) {
        block_centre_hyst -= 0.25 * block_centre_hyst.normalized();
        recompute = true;
    }
    else if (scale < last_scale && min_scale != -1) {
        block_centre_hyst += 0.25 * block_centre_hyst.normalized();
        recompute = true;
    }

    if (recompute) {
        return blockIntegrationScaleImpl(
            block_centre_hyst, map_res, last_scale, -1, max_block_scale);
    }
    else {
        return scale;
    }
}



bool se::Lidar::pointInFrustumImpl(const Eigen::Vector3f& point_S) const
{
    if (point_S.norm() > far_plane) {
        return false;
    }

    if (point_S.norm() < near_plane) {
        return false;
    }

    const float point_elevation = std::asin(point_S.z() / point_S.norm());

    if (point_elevation < min_elevation_rad || point_elevation > max_elevation_rad) {
        return false;
    }

    return true;
}



bool se::Lidar::pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const
{
    if (point_S.norm() < near_plane) {
        return false;
    }

    const float point_elevation = std::asin(point_S.z() / point_S.norm());

    if (point_elevation < min_elevation_rad || point_elevation > max_elevation_rad) {
        return false;
    }

    return true;
}



bool se::Lidar::sphereInFrustumImpl(const Eigen::Vector3f& centre_S, const float radius) const
{
    if (centre_S.norm() - radius > far_plane) {
        return false;
    }

    if (centre_S.norm() + radius < near_plane) {
        return false;
    }

    const float centre_elevation_rad = std::asin(centre_S.z() / centre_S.norm());

    if (centre_elevation_rad < min_elevation_rad) {
        const float delta_elevation = std::abs(centre_elevation_rad - min_elevation_rad);
        const float cone_dist = std::sin(delta_elevation) * centre_S.norm();
        if (cone_dist > radius) {
            return false;
        }
    }

    if (centre_elevation_rad > max_elevation_rad) {
        const float delta_elevation = std::abs(centre_elevation_rad - max_elevation_rad);
        const float cone_dist = std::sin(delta_elevation) * centre_S.norm();
        if (cone_dist > radius) {
            return false;
        }
    }

    return true;
}



bool se::Lidar::sphereInFrustumInfImpl(const Eigen::Vector3f& centre_S,
                                            const float radius) const
{
    if (centre_S.norm() + radius < near_plane) {
        return false;
    }

    const float centre_elevation_rad = std::asin(centre_S.z() / centre_S.norm());

    if (centre_elevation_rad < min_elevation_rad) {
        const float delta_elevation = std::abs(centre_elevation_rad - min_elevation_rad);
        const float cone_dist = std::sin(delta_elevation) * centre_S.norm();
        if (cone_dist > radius) {
            return false;
        }
    }

    if (centre_elevation_rad > max_elevation_rad) {
        const float delta_elevation = std::abs(centre_elevation_rad - max_elevation_rad);
        const float cone_dist = std::sin(delta_elevation) * centre_S.norm();
        if (cone_dist > radius) {
            return false;
        }
    }
    return true;
}
