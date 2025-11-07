/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SENSOR_HPP
#define SE_SENSOR_HPP

#include <limits>
#include <se/common/image_utils.hpp>
#include <se/common/math_util.hpp>
#include <se/common/projection.hpp>
#include <se/common/str_utils.hpp>
#include <se/common/yaml.hpp>
#include <se/image/image.hpp>

namespace se {

/** Base class for all sensor models used for integrating measurements. It uses CRTP for
 * compile-time polymorphism: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
 */
template<typename DerivedT>
class SensorBase {
    public:
    /** Configuration parameters common for all sensor models. */
    struct Config {
        /** The width of images produced by the sensor in pixels. */
        int width = 0;

        /** The height of images produced by the sensor in pixels. */
        int height = 0;

        /** The sensor's near plane in metres. Avoid setting to 0 since numerical issues may arise.
         */
        float near_plane = 0.01f;

        /** The sensor's far plane in metres. Avoid setting to infinity since performance may
         * degrade significantly, for example with depth images containing really large erroneous
         * measurements.
         */
        float far_plane = 10.0f;

        /** The transformation from the sensor frame S to the body frame B. */
        Eigen::Isometry3f T_BS = Eigen::Isometry3f::Identity();

        /** The pixel-size to voxel-size ratio thresholds, in ascendig order and in physical
         * coordinates, for computing the integration scale. See also
         * se::SensorBase::blockIntegrationScale(). For example:
         * - `pixel/voxel < pixel_voxel_ratio_per_scale[0]` → `scale = 0`
         * - `pixel/voxel < pixel_voxel_ratio_per_scale[1]` → `scale = 1`
         * - etc.
         */
        std::vector<float> pixel_voxel_ratio_per_scale = {1.5f, 3.0f, 6.0f};

        /** Reads the struct members from the "sensor" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& filename);

        Config operator/(const float downsampling_factor) const
        {
            return Config{static_cast<int>(width / downsampling_factor),
                          static_cast<int>(height / downsampling_factor),
                          near_plane,
                          far_plane,
                          T_BS,
                          pixel_voxel_ratio_per_scale};
        }

        // The definition of this function MUST be inside the definition of Config for template
        // argument deduction to work.
        friend std::ostream& operator<<(std::ostream& os, const Config& c)
        {
            os << str_utils::value_to_pretty_str(c.width, "width") << " px\n";
            os << str_utils::value_to_pretty_str(c.height, "height") << " px\n";
            os << str_utils::value_to_pretty_str(c.near_plane, "near_plane") << " m\n";
            os << str_utils::value_to_pretty_str(c.far_plane, "far_plane") << " m\n";
            os << str_utils::eigen_matrix_to_pretty_str(c.T_BS.matrix(), "T_BS") << "\n";
            return os;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<typename ConfigT>
    SensorBase(const ConfigT& c);

    SensorBase(const DerivedT& d);

    /** Return the integration scale for an se::Block at \p block_centre_S. The scale depends on the
     * back-projected pixel size in voxel space and the values in
     * se::SensorBase::Config::pixel_voxel_ratio_per_scale.
     *
     * \param[in] block_centre_S  The coordinates of the block centre expressed in the sensor frame.
     * \param[in] map_res         The resolution of the map in metres.
     * \param[in] last_scale      The block's current scale.
     * \param[in] min_scale       The minimum scale at which this block has been updated. It should
     *                            be -1 if the block hasn't been updated yet.
     * \param[in] max_block_scale The maximum possible scale for an se::Block.
     * \return The scale the block should be updated at.
     */
    int blockIntegrationScale(const Eigen::Vector3f& block_centre_S,
                              const float map_res,
                              const int last_scale,
                              const int min_scale,
                              const int max_block_scale) const;

    /** Return the minimum distance along \p ray_S, expressed in the sensor frame, that can be
     * measured.
     *
     * \note This may differ from se::SensorBase::Config::near_plane depending on the underlying
     * projection model.
     */
    float nearDist(const Eigen::Vector3f& ray_S) const;

    /** Return the maximum distance along \p ray_S, expressed in the sensor frame, that can be
     * measured.
     *
     * \note This may differ from se::SensorBase::Config::far_plane depending on the underlying
     * projection model.
     */
    float farDist(const Eigen::Vector3f& ray_S) const;

    /** Return the depth measurement that would result form \p point_S, expressed in the sensor
     * frame, being measured.
     *
     * \note For example, for a pinhole camera this would return the z-coordinate of \p point_S
     * whereas for a LiDAR it would return the norm of \p point_S.
     */
    float measurementFromPoint(const Eigen::Vector3f& point_S) const;

    /** Return whether \p point_S, expressed in the sensor frame, is inside the sensor frustum. */
    bool pointInFrustum(const Eigen::Vector3f& point_S) const;

    /** Return whether \p point_S, expressed in the sensor frame, is inside the sensor frustum but
     * without considering the far plane. */
    bool pointInFrustumInf(const Eigen::Vector3f& point_S) const;

    /** Return whether a sphere with \p centre_S and \p radius, expressed in the sensor frame, is at
     * least partially inside the sensor frustum. This is a fast approximate test that in rare cases
     * may return false positives.
     */
    bool sphereInFrustum(const Eigen::Vector3f& centre_S, const float radius) const;

    /** Return whether a sphere with \p centre_S and \p radius, expressed in the sensor frame, is at
     * least partially inside the sensor frustum but without considering the far plane. This is a
     * fast approximate test that in rare cases may return false positives.
     */
    bool sphereInFrustumInf(const Eigen::Vector3f& centre_S, const float radius) const;

    /** Return the underlying sensor type as a string. */
    static std::string type();

    bool left_hand_frame;
    float near_plane;
    float far_plane;
    Eigen::Isometry3f T_BS;
    std::vector<float> pixel_voxel_ratio_per_scale;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    // Make sure the derived class and the template parameter are the same (i.e. prevent class D1 : Base<D2>)
    SensorBase(){};
    friend DerivedT;

    // Simplify access to derived member functions
    const DerivedT* underlying() const;
};

} // namespace se

#include "impl/sensor_impl.hpp"
#include "lidar.hpp"
#include "range_image_lidar.hpp"
#include "pinhole_camera.hpp"

#endif // SE_SENSOR_HPP
