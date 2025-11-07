/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RANGEIMAGE_LIDAR_IMPL_HPP
#define SE_RANGEIMAGE_LIDAR_IMPL_HPP

namespace se {



inline float se::RangeImageLidar::nearDistImpl(const Eigen::Vector3f&) const
{
    return near_plane;
}



inline float se::RangeImageLidar::farDistImpl(const Eigen::Vector3f&) const
{
    return far_plane;
}



inline float se::RangeImageLidar::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
    return point_S.norm();
}



inline std::string se::RangeImageLidar::typeImpl()
{
    return "RangeImageLidar";
}



} // namespace se

#endif // SE_RANGEIMAGE_LIDAR_IMPL_HPP
