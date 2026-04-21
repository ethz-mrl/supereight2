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
namespace scale {

constexpr Scale from_size(const int octant_size)
{
    assert(math::is_power_of_two(octant_size));
    return math::log2_const(octant_size);
}



constexpr int to_size(const Scale octant_scale)
{
    assert(octant_scale >= 0);
    return 1 << octant_scale;
}



constexpr RGB to_colour(const Scale octant_scale)
{
    assert(octant_scale >= 0);
    if (static_cast<size_t>(octant_scale) < colours.size()) {
        return colours[octant_scale];
    }
    return colours.back();
}

} // namespace scale
} // namespace se

#endif // SE_COMMON_SCALE_IMPL_HPP
