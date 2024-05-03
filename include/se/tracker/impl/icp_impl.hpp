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

static inline void newReduce(const int block_idx,
                             float* output_data,
                             const Eigen::Vector2i& output_res,
                             Data* J_data,
                             const Eigen::Vector2i& J_res)
{
    float* sums = output_data + block_idx * 32;

    for (unsigned int i = 0; i < 32; ++i) {
        sums[i] = 0;
    }

    float sums0, sums1, sums2, sums3, sums4, sums5, sums6, sums7, sums8, sums9, sums10, sums11,
        sums12, sums13, sums14, sums15, sums16, sums17, sums18, sums19, sums20, sums21, sums22,
        sums23, sums24, sums25, sums26, sums27, sums28, sums29, sums30, sums31;
    sums0 = 0.0f;
    sums1 = 0.0f;
    sums2 = 0.0f;
    sums3 = 0.0f;
    sums4 = 0.0f;
    sums5 = 0.0f;
    sums6 = 0.0f;
    sums7 = 0.0f;
    sums8 = 0.0f;
    sums9 = 0.0f;
    sums10 = 0.0f;
    sums11 = 0.0f;
    sums12 = 0.0f;
    sums13 = 0.0f;
    sums14 = 0.0f;
    sums15 = 0.0f;
    sums16 = 0.0f;
    sums17 = 0.0f;
    sums18 = 0.0f;
    sums19 = 0.0f;
    sums20 = 0.0f;
    sums21 = 0.0f;
    sums22 = 0.0f;
    sums23 = 0.0f;
    sums24 = 0.0f;
    sums25 = 0.0f;
    sums26 = 0.0f;
    sums27 = 0.0f;
    sums28 = 0.0f;
    sums29 = 0.0f;
    sums30 = 0.0f;
    sums31 = 0.0f;

#pragma omp parallel for reduction(+:sums0,sums1,sums2,sums3,sums4,sums5,sums6,sums7,sums8,sums9,sums10,sums11,sums12,sums13,sums14,sums15,sums16,sums17,sums18,sums19,sums20,sums21,sums22,sums23,sums24,sums25,sums26,sums27,sums28,sums29,sums30,sums31)
    for (int y = block_idx; y < output_res.y(); y += 8) {
        for (int x = 0; x < output_res.x(); x++) {
            const Data& row = J_data[(x + y * J_res.x())]; // ...
            if (row.result < 1) {
                // accesses sums[28..31]
                /*(sums+28)[1]*/ sums29 += row.result == -4 ? 1 : 0;
                /*(sums+28)[2]*/ sums30 += row.result == -5 ? 1 : 0;
                /*(sums+28)[3]*/ sums31 += row.result > -4 ? 1 : 0;

                continue;
            }
            // Error part
            /*sums[0]*/ sums0 += row.error * row.error;

            // JTe part
            /*for(int i = 0; i < 6; ++i)
        sums[i+1] += row.error * row.J[i];*/
            sums1 += row.error * row.J[0];
            sums2 += row.error * row.J[1];
            sums3 += row.error * row.J[2];
            sums4 += row.error * row.J[3];
            sums5 += row.error * row.J[4];
            sums6 += row.error * row.J[5];

            // JTJ part, unfortunatly the double loop is not unrolled well...
            /*(sums+7)[0]*/ sums7 += row.J[0] * row.J[0];
            /*(sums+7)[1]*/ sums8 += row.J[0] * row.J[1];
            /*(sums+7)[2]*/ sums9 += row.J[0] * row.J[2];
            /*(sums+7)[3]*/ sums10 += row.J[0] * row.J[3];

            /*(sums+7)[4]*/ sums11 += row.J[0] * row.J[4];
            /*(sums+7)[5]*/ sums12 += row.J[0] * row.J[5];

            /*(sums+7)[6]*/ sums13 += row.J[1] * row.J[1];
            /*(sums+7)[7]*/ sums14 += row.J[1] * row.J[2];
            /*(sums+7)[8]*/ sums15 += row.J[1] * row.J[3];
            /*(sums+7)[9]*/ sums16 += row.J[1] * row.J[4];

            /*(sums+7)[10]*/ sums17 += row.J[1] * row.J[5];

            /*(sums+7)[11]*/ sums18 += row.J[2] * row.J[2];
            /*(sums+7)[12]*/ sums19 += row.J[2] * row.J[3];
            /*(sums+7)[13]*/ sums20 += row.J[2] * row.J[4];
            /*(sums+7)[14]*/ sums21 += row.J[2] * row.J[5];

            /*(sums+7)[15]*/ sums22 += row.J[3] * row.J[3];
            /*(sums+7)[16]*/ sums23 += row.J[3] * row.J[4];
            /*(sums+7)[17]*/ sums24 += row.J[3] * row.J[5];

            /*(sums+7)[18]*/ sums25 += row.J[4] * row.J[4];
            /*(sums+7)[19]*/ sums26 += row.J[4] * row.J[5];

            /*(sums+7)[20]*/ sums27 += row.J[5] * row.J[5];

            // extra info here
            /*(sums+28)[0]*/ sums28 += 1;
        }
    }
    sums[0] = sums0;
    sums[1] = sums1;
    sums[2] = sums2;
    sums[3] = sums3;
    sums[4] = sums4;
    sums[5] = sums5;
    sums[6] = sums6;
    sums[7] = sums7;
    sums[8] = sums8;
    sums[9] = sums9;
    sums[10] = sums10;
    sums[11] = sums11;
    sums[12] = sums12;
    sums[13] = sums13;
    sums[14] = sums14;
    sums[15] = sums15;
    sums[16] = sums16;
    sums[17] = sums17;
    sums[18] = sums18;
    sums[19] = sums19;
    sums[20] = sums20;
    sums[21] = sums21;
    sums[22] = sums22;
    sums[23] = sums23;
    sums[24] = sums24;
    sums[25] = sums25;
    sums[26] = sums26;
    sums[27] = sums27;
    sums[28] = sums28;
    sums[29] = sums29;
    sums[30] = sums30;
    sums[31] = sums31;
}



static inline void reduceKernel(float* output_data,
                                const Eigen::Vector2i& output_res,
                                Data* J_data,
                                const Eigen::Vector2i& J_res)
{
#ifdef OLDREDUCE
#    pragma omp parallel for
#endif
    for (int block_idx = 0; block_idx < 8; block_idx++) {
        newReduce(block_idx, output_data, output_res, J_data, J_res);
    }

    Eigen::Map<Eigen::Matrix<float, 8, 32, Eigen::RowMajor>> values(output_data);
    for (int j = 1; j < 8; ++j) {
        values.row(0) += values.row(j);
    }
}



template<typename ProjectF>
void trackKernel(Data* output_data,
                 const se::Image<Eigen::Vector3f>& input_point_cloud_S,
                 const se::Image<Eigen::Vector3f>& input_normals_S,
                 const se::Image<Eigen::Vector3f>& surface_point_cloud_W_ref,
                 const se::Image<Eigen::Vector3f>& surface_normals_W_ref,
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

            const Eigen::Vector2i ref_pixel = se::round_pixel(ref_pixel_f);
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



static inline bool updatePoseKernel(Eigen::Isometry3f& T_WS,
                                    const float* reduction_output_data,
                                    const float icp_threshold)
{
    bool result = false;
    Eigen::Map<const Eigen::Matrix<float, 8, 32, Eigen::RowMajor>> values(reduction_output_data);
    Eigen::Matrix<float, 6, 1> x = solve(values.row(0).segment(1, 27));
    const Eigen::Isometry3f delta(se::math::exp(x));
    T_WS = delta * T_WS;

    if (x.norm() < icp_threshold) {
        result = true;
    }

    return result;
}



static inline bool checkPoseKernel(Eigen::Isometry3f& T_WS,
                                   Eigen::Isometry3f& previous_T_WS,
                                   const float* reduction_output_data,
                                   const Eigen::Vector2i& reduction_output_res,
                                   const float track_threshold)
{
    // Check the tracking result, and go back to the previous camera position if necessary

    const Eigen::Matrix<float, 8, 32, Eigen::RowMajor> values(reduction_output_data);

    if ((std::sqrt(values(0, 0) / values(0, 28)) > 2e-2)
        || (values(0, 28) / (reduction_output_res.x() * reduction_output_res.y())
            < track_threshold)) {
        T_WS = previous_T_WS;
        return false;
    }
    else {
        return true;
    }
}

} // namespace icp
} // namespace se

#endif // SE_ICP_IMPL_HPP
