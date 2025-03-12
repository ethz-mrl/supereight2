/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_LEICA_LIDAR_HPP
#define SE_LEICA_LIDAR_HPP



namespace se {

class LeicaLidar : public SensorBase<LeicaLidar> {
    public:
    struct Config : public SensorBase<LeicaLidar>::Config {
        /** \brief The actual sensor resolution in angular directions (in degrees).
         *  It is needed for ray based integration to decide the integration scale.
         */
        float elevation_resolution_angle_ = 1.0f;
        float azimuth_resolution_angle_ = 1.0f;

        /** Reads the struct members from the "sensor" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         *
         * \throws std::invalid_argument Throws std::invalid_argument if the number of beam elevation or
         * azimuth angles is different than the sensor height.
         */
        void readYaml(const std::string& filename);

        Config operator/(const float downsampling_factor) const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    LeicaLidar(const Config& config);

    LeicaLidar(const Config& config, const float downsampling_factor);

    LeicaLidar(const LeicaLidar& leica_lidar, const float downsampling_factor);

    int computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
                                    const float map_res,
                                    const int last_scale,
                                    const int min_scale,
                                    const int max_block_scale) const;

    float nearDistImpl(const Eigen::Vector3f& ray_S) const;

    float farDistImpl(const Eigen::Vector3f& ray_S) const;

    float measurementFromPointImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const;

    bool sphereInFrustumImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    bool sphereInFrustumInfImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    static std::string typeImpl();

    srl::projection::LeicaLidar model;
    /** \brief the minimum ray angle between subsequent measurements*/
    float min_ray_angle;
    float min_elevation_rad;
    float max_elevation_rad;
    /** \brief The horizontal field of view in radians. */
    float horizontal_fov;
    /** \brief The vertical field of view in radians. */
    float vertical_fov;

    /** \brief Angular resolution for ray Integration */
    float azimuth_resolution_angle;
    float elevation_resolution_angle;

    float pixel_dim_tan = 0.0f;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const LeicaLidar::Config& c);

} // namespace se

#include "impl/leica_lidar_impl.hpp"

#endif // SE_LEICA_LIDAR_HPP
