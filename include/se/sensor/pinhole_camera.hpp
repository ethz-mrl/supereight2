/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PINHOLE_CAMERA_HPP
#define SE_PINHOLE_CAMERA_HPP



namespace se {

class PinholeCamera : public SensorBase<PinholeCamera> {
    public:
    struct Config : public SensorBase<PinholeCamera>::Config {
        /** The sensor's horizontal focal length in pixels.
         */
        float fx = std::numeric_limits<float>::quiet_NaN();

        /** The sensor's vertical focal length in pixels.
         */
        float fy = std::numeric_limits<float>::quiet_NaN();

        /** The sensor's optical centre horizontal coordinate in pixels.
         */
        float cx = std::numeric_limits<float>::quiet_NaN();

        /** The sensor's optical centre vertical coordinate in pixels.
         */
        float cy = std::numeric_limits<float>::quiet_NaN();

        /** Reads the struct members from the "sensor" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& filename);

        Config operator/(const float downsampling_factor) const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** Linear indices to se::PinholeCamera::frustum_vertices_S and number of frustum vertices. */
    struct FrustumVertex {
        enum {
            TopLeftNear,
            TopRightNear,
            BottomRightNear,
            BottomLeftNear,
            TopLeftFar,
            TopRightFar,
            BottomRightFar,
            BottomLeftFar,
            Num,
        };
    };

    /** Linear indices to se::PinholeCamera::frustum_normals_S and number of frustum normals. */
    struct FrustumNormal {
        enum {
            Left,
            Right,
            Bottom,
            Top,
            Near,
            Far,
            Num,
        };
    };



    PinholeCamera(const Config& config);

    PinholeCamera(const Config& config, const float downsampling_factor);

    PinholeCamera(const PinholeCamera& pinhole_camera, const float downsampling_factor);

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

    /** Return an se::PinholeCamera instance for use in unit tests requiring one. */
    static PinholeCamera testInstance();

    srl::projection::PinholeCamera<srl::projection::NoDistortion> model;
    float scaled_pixel;

    /** \brief The horizontal field of view in radians. */
    float horizontal_fov;

    /** \brief The vertical field of view in radians. */
    float vertical_fov;

    /** The vertices of the camera frustum expressed in the camera frame S. */
    Eigen::Matrix<float, 3, FrustumVertex::Num> frustum_vertices_S;

    /** The inwards-pointing normals of the camera frustum faces expressed in the camera frame S. */
    Eigen::Matrix<float, 4, FrustumNormal::Num> frustum_normals_S;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    void computeFrustumVertices();
    void computeFrustumNormals();
};

std::ostream& operator<<(std::ostream& os, const PinholeCamera::Config& c);

} // namespace se

#include "impl/pinhole_camera_impl.hpp"

#endif // SE_PINHOLE_CAMERA_HPP
