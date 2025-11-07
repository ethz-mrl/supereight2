/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RANGEIMAGE_LIDAR_HPP
#define SE_RANGEIMAGE_LIDAR_HPP



namespace se {

class RangeImageLidar : public SensorBase<RangeImageLidar> {
    public:
    struct Config : public SensorBase<RangeImageLidar>::Config {
        /** The elevation offset for each Lidar beam in degrees. The number of offsets should be the
         * same as se::SensorBase<RangeImageLidar>::Config::height.
         */
        Eigen::VectorXf beam_elevation_angles = Eigen::VectorXf(1);

        /** The azimuth offset for each Lidar beam in degrees. The number of offsets should be the same
         * as se::SensorBase<RangeImageLidar>::Config::height.
         */
        Eigen::VectorXf beam_azimuth_angles = Eigen::VectorXf(1);

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

    RangeImageLidar(const Config& config);

    RangeImageLidar(const Config& config, const float downsampling_factor);

    RangeImageLidar(const RangeImageLidar& range_image_lidar, const float downsampling_factor);

    Eigen::Matrix3f K() const
    {
        return Eigen::Matrix3f::Zero();
    }

    int blockIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
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

    srl::projection::RangeImageLidar model;
    float min_ray_angle;

    float min_elevation_rad;
    float max_elevation_rad;

    /** \brief The horizontal field of view in radians. */
    float horizontal_fov;
    /** \brief The vertical field of view in radians. */
    float vertical_fov;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const RangeImageLidar::Config& c);

} // namespace se

#include "impl/range_image_lidar_impl.hpp"

#endif // SE_RANGEIMAGE_LIDAR_HPP
