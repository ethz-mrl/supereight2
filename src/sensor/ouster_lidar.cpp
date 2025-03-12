/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/sensor/sensor.hpp"

void se::OusterLidar::Config::readYaml(const std::string& filename)
{
    // Read the base class members.
    SensorBase<OusterLidar>::Config::readYaml(filename);

    // Open the file for reading.
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];

    se::yaml::subnode_as_eigen_vector_x(node, "elevation_angles", beam_elevation_angles);
    if (beam_elevation_angles.size() != height) {
        throw std::invalid_argument("expected " + std::to_string(height)
                                    + " beam elevation angles but got "
                                    + std::to_string(beam_elevation_angles.size()));
    }
    se::yaml::subnode_as_eigen_vector_x(node, "azimuth_angles", beam_azimuth_angles);
    if (beam_azimuth_angles.size() != height) {
        throw std::invalid_argument("expected " + std::to_string(height)
                                    + " beam azimuth angles but got "
                                    + std::to_string(beam_azimuth_angles.size()));
    }
}

se::OusterLidar::Config se::OusterLidar::Config::operator/(const float downsampling_factor) const
{
    return se::OusterLidar::Config{
        se::SensorBase<se::OusterLidar>::Config::operator/(downsampling_factor),
        beam_elevation_angles,
        beam_azimuth_angles,
    };
}



std::ostream& se::operator<<(std::ostream& os, const se::OusterLidar::Config& c)
{
    os << static_cast<const SensorBase<OusterLidar>::Config&>(c);
    os << str_utils::eigen_vector_to_pretty_str(c.beam_azimuth_angles, "beam_azimuth_angles")
       << " degrees\n";
    os << str_utils::eigen_vector_to_pretty_str(c.beam_elevation_angles, "beam_elevation_angles")
       << " degrees\n";
    return os;
}



se::OusterLidar::OusterLidar(const Config& c) :
        se::SensorBase<se::OusterLidar>(c),
        model(c.width, c.height, c.beam_azimuth_angles, c.beam_elevation_angles)
{
    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane > 0.0f);
    assert(c.far_plane > c.near_plane);
    assert(c.beam_azimuth_angles.size() == c.height);
    assert(c.beam_elevation_angles.size() == c.height);
    float min_elevation_angle = fabsf(c.beam_elevation_angles[1] - c.beam_elevation_angles[0]);
    for (int i = 2; i < c.beam_elevation_angles.size(); i++) {
        const float diff = fabsf(c.beam_elevation_angles[i - 1] - c.beam_elevation_angles[i]);
        if (diff < min_elevation_angle) {
            min_elevation_angle = diff;
        }
    }
    const float azimuth_angle = 360.0f / c.width;
    min_ray_angle = std::min(min_elevation_angle, azimuth_angle);
    horizontal_fov = 2.0f * M_PI;
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float min_elevation = c.beam_elevation_angles.minCoeff();
    min_elevation_rad = min_elevation * deg_to_rad;
    const float max_elevation = c.beam_elevation_angles.maxCoeff();
    max_elevation_rad = max_elevation * deg_to_rad;
    vertical_fov = deg_to_rad * (max_elevation - min_elevation);
}



se::OusterLidar::OusterLidar(const Config& c, const float dsf) :
        se::SensorBase<se::OusterLidar>(c),
        model(c.width / dsf, c.height / dsf, c.beam_azimuth_angles, c.beam_elevation_angles)
{
    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane >= 0.f);
    assert(c.far_plane > c.near_plane);
    assert(c.beam_azimuth_angles.size() > 0);
    assert(c.beam_elevation_angles.size() > 0);
    float min_elevation_angle = fabsf(c.beam_elevation_angles[1] - c.beam_elevation_angles[0]);
    for (int i = 2; i < c.beam_elevation_angles.size(); i++) {
        const float diff = fabsf(c.beam_elevation_angles[i - 1] - c.beam_elevation_angles[i]);
        if (diff < min_elevation_angle) {
            min_elevation_angle = diff;
        }
    }
    const float azimuth_angle = 360.0f / c.width;
    min_ray_angle = std::min(min_elevation_angle, azimuth_angle);
    horizontal_fov = 2.0f * M_PI;
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float min_elevation = c.beam_elevation_angles.minCoeff();
    min_elevation_rad = min_elevation * deg_to_rad;
    const float max_elevation = c.beam_elevation_angles.maxCoeff();
    max_elevation_rad = max_elevation * deg_to_rad;
    vertical_fov = deg_to_rad * (max_elevation - min_elevation);
}



se::OusterLidar::OusterLidar(const OusterLidar& ol, const float dsf) :
        se::SensorBase<se::OusterLidar>(ol),
        model(ol.model.imageWidth() / dsf,
              ol.model.imageHeight() / dsf,
              ol.model.beamAzimuthAngles(),
              ol.model.beamElevationAngles()), // TODO: Does the beam need to be scaled too?
        min_ray_angle(ol.min_ray_angle),
        min_elevation_rad(ol.min_elevation_rad),
        max_elevation_rad(ol.max_elevation_rad),
        horizontal_fov(ol.horizontal_fov),
        vertical_fov(ol.vertical_fov)
{
}



int se::OusterLidar::computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
                                                 const float map_res,
                                                 const int last_scale,
                                                 const int min_scale,
                                                 const int max_block_scale) const
{
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float dist = block_centre.norm();
    // Compute the side length in metres of a pixel projected dist metres from
    // the camera. This computes the chord length corresponding to the ray angle
    // at distance dist.
    const float pixel_dim = 2.0f * dist * std::tan(min_ray_angle / 2.0f * deg_to_rad);
    // Compute the ratio using the worst case voxel_dim (space diagonal)
    const float pv_ratio = pixel_dim / (std::sqrt(3) * map_res);
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
        return computeIntegrationScaleImpl(
            block_centre_hyst, map_res, last_scale, -1, max_block_scale);
    }
    else {
        return scale;
    }
}



bool se::OusterLidar::pointInFrustumImpl(const Eigen::Vector3f& point_S) const
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



bool se::OusterLidar::pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const
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



bool se::OusterLidar::sphereInFrustumImpl(const Eigen::Vector3f& centre_S, const float radius) const
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



bool se::OusterLidar::sphereInFrustumInfImpl(const Eigen::Vector3f& centre_S,
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
