/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_SCALE_HPP
#define SE_COMMON_SCALE_HPP

#include <se/common/math_util.hpp>

namespace se {

/** The scale of some octree volume is in a sense the inverse of its depth. Where the coarsest
 * octree volume has a depth of 0, it is the finest octree volume that has a scale of 0. This makes
 * the scale a useful way to specify a desired resolution, irrespective of the number of levels in
 * the octree.
 *
 * | Volume size (voxels) | Scale              |
 * |:--------------------:|:------------------:|
 * | 1×1×1                | 0                  |
 * | 2×2×2                | 1                  |
 * | 4×4×4                | 2                  |
 * | 8×8×8                | 3                  |
 * | 16×16×16             | 4                  |
 * | ...                  | ...                |
 * | NxNxN                | log<sub>2</sub>(N) |
 */
typedef int Scale;



namespace octantops {

/** Return the octree scale corresponding to \p octant_size in voxels. */
constexpr Scale size_to_scale(const int octant_size);

/** Return the octant size in voxels corresponding to \p octant_scale. */
constexpr int scale_to_size(const Scale octant_scale);

} // namespace octantops

} // namespace se

#include "impl/scale_impl.hpp"

#endif // SE_COMMON_SCALE_HPP
