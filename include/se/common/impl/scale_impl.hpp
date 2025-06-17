/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_SCALE_IMPL_HPP
#define SE_COMMON_SCALE_IMPL_HPP

namespace se {
namespace octantops {

constexpr Scale size_to_scale(const int octant_size)
{
    assert(math::is_power_of_two(octant_size));
    return math::log2_const(octant_size);
}



constexpr int scale_to_size(const Scale octant_scale)
{
    assert(octant_scale >= 0);
    return 1 << octant_scale;
}

} // namespace octantops
} // namespace se

#endif // SE_COMMON_SCALE_IMPL_HPP
