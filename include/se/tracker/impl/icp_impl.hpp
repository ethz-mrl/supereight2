/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_ICP_IMPL_HPP
#define SE_ICP_IMPL_HPP

namespace se {
namespace icp {

template<typename ProjectF>
void trackKernel(Data* output_data,
                 const Image<Eigen::Vector3f>& input_point_cloud_S,
                 const Image<Eigen::Vector3f>& input_normals_S,
                 const Image<Eigen::Vector3f>& surface_point_cloud_W_ref,
                 const Image<Eigen::Vector3f>& surface_normals_W_ref,
                 const Eigen::Isometry3f& T_WS,
                 const Eigen::Isometry3f& T_WS_ref,
                 const ProjectF project,
                 const float dist_threshold,
                 const float normal_threshold)
{
    const Eigen::Vector2i input_res(input_point_cloud_S.width(), input_point_cloud_S.height());
    const Eigen::Vector2i ref_res(surface_point_cloud_W_ref.width(),
                                  surface_point_cloud_W_ref.height());

    const int h = input_res.y(); // clang complains if this is inside the for loop
    const int w = input_res.x(); // clang complains if this is inside the for loop
#pragma omp parallel for
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const Eigen::Vector2i pixel(x, y);

            Data& row = output_data[pixel.x() + pixel.y() * ref_res.x()];

            if (input_normals_S[pixel.x() + pixel.y() * w] == math::g_invalid_normal) {
                row.result = -1;
                continue;
            }

            // point_W := The input point in world frame
            const Eigen::Vector3f point_W = T_WS * input_point_cloud_S[pixel.x() + pixel.y() * w];
            // point_S_ref := The input point expressed in the sensor frame the
            // surface_point_cloud_W_ref and surface_point_cloud_W_ref was raycasted from.
            const Eigen::Vector3f point_S_ref = T_WS_ref.inverse() * point_W;

            // ref_pixel_f := The pixel in the surface_point_cloud_M_ref and surface_point_cloud_M_ref image.
            Eigen::Vector2f ref_pixel_f;
            // TODO
            //if (sensor_.model.project(point_S_ref, &ref_pixel_f)
            //    != srl::projection::ProjectionStatus::Successful) {
            if (!project(point_S_ref, ref_pixel_f)) {
                row.result = -2;
                continue;
            }

            const Eigen::Vector2i ref_pixel = round_pixel(ref_pixel_f);
            // TODO: Properly fix the ICP tracking instead of blindly inverting the reference
            // normal. Due to a bug in raycast_volume(), it used to return the inwards instead of
            // the outwards facing normals. Using the outwards facing normals breaks tracking in the
            // TUM RGB-D dataset but it's not clear why. Added the normal inversion here until a
            // proper fix for the ICP is found.
            const Eigen::Vector3f ref_normal_W =
                -surface_normals_W_ref[ref_pixel.x() + ref_pixel.y() * ref_res.x()];

            if (ref_normal_W == math::g_invalid_normal) {
                row.result = -3;
                continue;
            }

            const Eigen::Vector3f ref_point_W =
                surface_point_cloud_W_ref[ref_pixel.x() + ref_pixel.y() * ref_res.x()];
            const Eigen::Vector3f diff = ref_point_W - point_W;
            const Eigen::Vector3f input_normal_W =
                T_WS.linear() * input_normals_S[pixel.x() + pixel.y() * w];

            if (diff.norm() > dist_threshold) {
                row.result = -4;
                continue;
            }
            if (input_normal_W.dot(ref_normal_W) < normal_threshold) {
                row.result = -5;
                continue;
            }
            row.result = 1;
            row.error = ref_normal_W.dot(diff);
            row.J[0] = ref_normal_W.x();
            row.J[1] = ref_normal_W.y();
            row.J[2] = ref_normal_W.z();

            const Eigen::Vector3f cross_prod = point_W.cross(ref_normal_W);
            row.J[3] = cross_prod.x();
            row.J[4] = cross_prod.y();
            row.J[5] = cross_prod.z();
        }
    }
}

} // namespace icp
} // namespace se

#endif // SE_ICP_IMPL_HPP
