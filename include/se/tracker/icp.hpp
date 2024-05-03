/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_ICP_HPP
#define SE_ICP_HPP

#include <Eigen/Geometry>
#include <se/common/image_utils.hpp>
#include <se/common/math_util.hpp>
#include <se/image/image.hpp>

namespace se {
namespace icp {

struct Data {
    int result;
    float error;
    float J[6];

    Data() : result(0), error(0.0f), J{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    {
    }
};



static inline Eigen::Matrix<float, 6, 6> makeJTJ(const Eigen::Matrix<float, 1, 21>& v)
{
    Eigen::Matrix<float, 6, 6> C = Eigen::Matrix<float, 6, 6>::Zero();
    C.row(0) = v.segment(0, 6);
    C.row(1).segment(1, 5) = v.segment(6, 5);
    C.row(2).segment(2, 4) = v.segment(11, 4);
    C.row(3).segment(3, 3) = v.segment(15, 3);
    C.row(4).segment(4, 2) = v.segment(18, 2);
    C(5, 5) = v(20);

    for (int r = 1; r < 6; ++r)
        for (int c = 0; c < r; ++c)
            C(r, c) = C(c, r);
    return C;
}



static inline Eigen::Matrix<float, 6, 1> solve(const Eigen::Matrix<float, 1, 27>& vals)
{
    const Eigen::Matrix<float, 6, 1> b = vals.segment(0, 6);
    const Eigen::Matrix<float, 6, 6> C = makeJTJ(vals.segment(6, 21));
    Eigen::LLT<Eigen::Matrix<float, 6, 6>> llt;
    llt.compute(C);
    Eigen::Matrix<float, 6, 1> res = llt.solve(b);
    return llt.info() == Eigen::Success ? res : Eigen::Matrix<float, 6, 1>::Zero();
}



static inline void newReduce(const int block_idx,
                             float* output_data,
                             const Eigen::Vector2i& output_res,
                             Data* J_data,
                             const Eigen::Vector2i& J_res);

static inline void reduceKernel(float* output_data,
                                const Eigen::Vector2i& output_res,
                                Data* J_data,
                                const Eigen::Vector2i& J_res);

/**
 * ProjectF is functor with the following declaration, returning whether the projection of point_S
 * succeeded:
 *\code{.cpp}
 *bool project(const Eigen::Vector3f& point_S, Eigen::Vector2f& pixel);
 *\endcode
 */
template<typename ProjectF>
void trackKernel(Data* output_data,
                 const se::Image<Eigen::Vector3f>& input_point_cloud_S,
                 const se::Image<Eigen::Vector3f>& input_normals_S,
                 const se::Image<Eigen::Vector3f>& surface_point_cloud_M_ref,
                 const se::Image<Eigen::Vector3f>& surface_normals_M_ref,
                 const Eigen::Isometry3f& T_WS,
                 const Eigen::Isometry3f& T_WS_ref,
                 const ProjectF project,
                 const float dist_threshold,
                 const float normal_threshold);

static inline bool updatePoseKernel(Eigen::Isometry3f& T_WS,
                                    const float* reduction_output_data,
                                    const float icp_threshold);

static inline bool checkPoseKernel(Eigen::Isometry3f& T_WS,
                                   Eigen::Isometry3f& previous_T_WS,
                                   const float* reduction_output_data,
                                   const Eigen::Vector2i& reduction_output_res,
                                   const float track_threshold);

} // namespace icp
} // namespace se

#include "impl/icp_impl.hpp"

#endif // SE_ICP_HPP
