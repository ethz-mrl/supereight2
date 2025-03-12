/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_TRACKER_IMPL_HPP
#define SE_TRACKER_IMPL_HPP

namespace se {



template<typename MapT, typename SensorT>
bool Tracker<MapT, SensorT>::track(const Image<float>& depth_img, Eigen::Isometry3f& T_WS)
{
    Image<Eigen::Vector3f> surface_point_cloud_M(
        depth_img.width(), depth_img.height(), Eigen::Vector3f::Zero());
    Image<Eigen::Vector3f> surface_normals_M(
        depth_img.width(), depth_img.height(), Eigen::Vector3f::Zero());
    raycaster::raycast_volume(map_, sensor_, T_WS, surface_point_cloud_M, surface_normals_M);
    return track(depth_img, T_WS, surface_point_cloud_M, surface_normals_M);
}



template<typename MapT, typename SensorT>
bool Tracker<MapT, SensorT>::track(const Image<float>& depth_img,
                                   Eigen::Isometry3f& T_WS,
                                   Image<Eigen::Vector3f>& surface_point_cloud_W,
                                   Image<Eigen::Vector3f>& surface_normals_W)
{
    assert(depth_img.width() == surface_point_cloud_W.width()
           && depth_img.height() == surface_point_cloud_W.height());
    assert(depth_img.width() == surface_normals_W.width()
           && depth_img.height() == surface_normals_W.height());

    Eigen::Isometry3f T_WS_ref = T_WS; // Camera pose of the previous image in world frame

    Eigen::Vector2i depth_img_res = Eigen::Vector2i(depth_img.width(), depth_img.height());

    std::vector<Image<float>> scaled_depth_img;
    std::vector<float> reduction_output(8 * 32, 0.0f);
    std::vector<Image<Eigen::Vector3f>> input_point_cloud_S; //< Point cloud in sensor frame
    std::vector<Image<Eigen::Vector3f>> input_normals_S;     //< Normals in sensor frame

    // Initialize the scaled images
    for (unsigned int i = 0; i < config_.iterations.size(); ++i) {
        const int downsample = 1 << i;
        const Eigen::Vector2i scaled_res =
            Eigen::Vector2i(sensor_.model.imageWidth(), sensor_.model.imageHeight()) / downsample;
        scaled_depth_img.emplace_back(scaled_res.x(), scaled_res.y(), 0.0f);
        input_point_cloud_S.emplace_back(scaled_res.x(), scaled_res.y(), Eigen::Vector3f::Zero());
        input_normals_S.emplace_back(scaled_res.x(), scaled_res.y(), Eigen::Vector3f::Zero());
    }

    std::memcpy(scaled_depth_img[0].data(),
                depth_img.data(),
                sizeof(float) * depth_img.width() * depth_img.height());

    // Half sample the input depth maps into the pyramid levels
    for (unsigned int i = 1; i < config_.iterations.size(); ++i) {
        preprocessor::half_sample_robust_image(
            scaled_depth_img[i], scaled_depth_img[i - 1], e_delta * 3, 1);
    }

    // Prepare the 3D information from the input depth maps
    for (unsigned int i = 0; i < config_.iterations.size(); ++i) {
        const float scaling_factor = 1 << i;
        const SensorT scaled_sensor(sensor_, scaling_factor);
        preprocessor::depth_to_point_cloud(
            input_point_cloud_S[i], scaled_depth_img[i], scaled_sensor);
        if (sensor_.left_hand_frame) {
            preprocessor::point_cloud_to_normal<true>(input_normals_S[i], input_point_cloud_S[i]);
        }
        else {
            preprocessor::point_cloud_to_normal<false>(input_normals_S[i], input_point_cloud_S[i]);
        }
    }

    for (int level = config_.iterations.size() - 1; level >= 0; --level) {
        Eigen::Vector2i reduction_output_res(depth_img_res.x() / (int) pow(2, level),
                                             depth_img_res.y() / (int) pow(2, level));

        for (int i = 0; i < config_.iterations[level]; ++i) {
            icp::trackKernel(
                tracking_result.data(),
                input_point_cloud_S[level],
                input_normals_S[level],
                surface_point_cloud_W,
                surface_normals_W,
                T_WS,
                T_WS_ref,
                [&](const Eigen::Vector3f& point_S, Eigen::Vector2f& pixel) {
                    return sensor_.model.project(point_S, &pixel)
                        == srl::projection::ProjectionStatus::Successful;
                },
                config_.dist_threshold,
                config_.normal_threshold);

            icp::reduceKernel(reduction_output.data(),
                              reduction_output_res,
                              tracking_result.data(),
                              depth_img_res);

            if (icp::updatePoseKernel(T_WS, reduction_output.data(), config_.icp_threshold)) {
                break;
            }
        }
    }
    return icp::checkPoseKernel(
        T_WS, T_WS_ref, reduction_output.data(), depth_img_res, config_.track_threshold);
}



template<typename MapT, typename SensorT>
void Tracker<MapT, SensorT>::renderTrackingResult(RGBA* tracking_img_data)
{
#pragma omp parallel for
    for (int y = 0; y < sensor_.model.imageHeight(); y++) {
        for (int x = 0; x < sensor_.model.imageWidth(); x++) {
            const int pixel_idx = x + sensor_.model.imageWidth() * y;
            switch (tracking_result[pixel_idx].result) {
            case icp::ResultSuccess:
                // Gray
                tracking_img_data[pixel_idx] = {0x80, 0x80, 0x80, 0xFF};
                break;
            case icp::ResultInvalidInputNormal:
                // Black
                tracking_img_data[pixel_idx] = {0x00, 0x00, 0x00, 0xFF};
                break;
            case icp::ResultProjectionOutside:
                // Red
                tracking_img_data[pixel_idx] = {0xFF, 0x00, 0x00, 0xFF};
                break;
            case icp::ResultInvalidRefNormal:
                // Green
                tracking_img_data[pixel_idx] = {0x00, 0xFF, 0x00, 0xFF};
                break;
            case icp::ResultDistThreshold:
                // Blue
                tracking_img_data[pixel_idx] = {0x00, 0x00, 0xFF, 0xFF};
                break;
            case icp::ResultNormalThreshold:
                // Yellow
                tracking_img_data[pixel_idx] = {0xFF, 0xFF, 0x00, 0xFF};
                break;
            default:
                // Orange
                tracking_img_data[pixel_idx] = {0xFF, 0x80, 0x80, 0xFF};
                break;
            }
        }
    }
}

} // namespace se

#endif // SE_TRACKER_IMPL_HPP
