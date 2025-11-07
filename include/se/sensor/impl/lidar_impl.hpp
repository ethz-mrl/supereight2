/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_LIDAR_IMPL_HPP
#define SE_LIDAR_IMPL_HPP

namespace se {



inline float se::Lidar::nearDistImpl(const Eigen::Vector3f&) const
{
    return near_plane;
}



inline float se::Lidar::farDistImpl(const Eigen::Vector3f&) const
{
    return far_plane;
}



inline float se::Lidar::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
    return point_S.norm();
}



inline std::string se::Lidar::typeImpl()
{
    return "Lidar";
}



} // namespace se

#endif // SE_LIDAR_IMPL_HPP
