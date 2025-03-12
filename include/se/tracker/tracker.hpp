/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_TRACKER_HPP
#define SE_TRACKER_HPP

#include <se/map/preprocessor.hpp>
#include <se/map/raycaster.hpp>
#include <se/sensor/sensor.hpp>
#include <se/tracker/icp.hpp>

namespace se {

constexpr float e_delta = 0.1f;

struct TrackerConfig {
    std::vector<int> iterations{10, 5, 4};
    float dist_threshold = 0.1f;
    float normal_threshold = 0.8f;
    float track_threshold = 0.15f;
    float icp_threshold = 0.00001f;

    /** Reads the struct members from the "tracker" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const TrackerConfig& c);



template<typename MapT, typename SensorT>
class Tracker {
    public:
    Tracker(MapT& map, const SensorT& sensor, const TrackerConfig config = TrackerConfig()) :
            map_(map),
            sensor_(sensor),
            config_(config),
            tracking_result(sensor_.model.imageWidth() * sensor_.model.imageHeight(), icp::Data())
    {
    }

    /**
     * \brief Track the current pose using ICP.
     *
     * \param[in]     depth_image_    The depth image fitting the sensor model
     * \param[in,out] T_WS            [in] The previous best pose estimate, [out] The new best pose estimate
     *
     * \return The tracking success.
     */
    bool track(const Image<float>& depth_img, Eigen::Isometry3f& T_WS);

    /**
     * \brief Track the current pose using ICP.
     *
     * \param[in]     depth_image_            The depth image fitting the sensor model
     * \param[in,out] T_WS                    [in] The previous best pose estimate T_WS_ref, [out] The new best pose estimate T_WS
     * \param[in]     surface_point_cloud_W   Surface point cloud in world frame
     * \param[in]     surface_normals_W       Surface normals in world frame
     *
     * \return The tracking success.
     */
    bool track(const Image<float>& depth_img,
               Eigen::Isometry3f& T_WS,
               Image<Eigen::Vector3f>& surface_point_cloud_W,
               Image<Eigen::Vector3f>& surface_normals_W);

    void renderTrackingResult(RGBA* tracking_img_data);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    MapT& map_;
    const SensorT& sensor_;
    const TrackerConfig config_;
    std::vector<icp::Data> tracking_result;
};

} // namespace se

#include "impl/tracker_impl.hpp"

#endif // SE_TRACKER_HPP
