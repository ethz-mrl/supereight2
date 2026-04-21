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
#include <se/common/rgb.hpp>

namespace se {

/** The scale of some octree volume is in a sense the inverse of its depth. Where the coarsest
 * octree volume has a depth of 0, it is the finest octree volume that has a scale of 0. This makes
 * the scale a useful way to specify a desired resolution, irrespective of the number of levels in
 * the octree.
 *
 * | Volume size (voxels)                      | Scale              |
 * |:-----------------------------------------:|:------------------:|
 * | 1×1×1                                     | 0                  |
 * | 2×2×2                                     | 1                  |
 * | 4×4×4                                     | 2                  |
 * | 8×8×8                                     | 3                  |
 * | 16×16×16                                  | 4                  |
 * | ...                                       | ...                |
 * | N×N×N                                     | log<sub>2</sub>(N) |
 * | 2<sup>M</sup>×2<sup>M</sup>×2<sup>M</sup> | M                  |
 *
 * See se::colours::scale for the colours used to visualize each scale.
 */
typedef int Scale;



/** Operations on se::Scale. */
namespace scale {

/** The colours used to visualize each value of se::Scale.
 *
 * <table>
 * <tr><th>Scale</th><th>Colour</th></tr>
 * <tr><td>0</td><td style="color:#66c2a5">████</td></tr>
 * <tr><td>1</td><td style="color:#fc8d62">████</td></tr>
 * <tr><td>2</td><td style="color:#8da0cb">████</td></tr>
 * <tr><td>3</td><td style="color:#e78ac3">████</td></tr>
 * <tr><td>4</td><td style="color:#a6d854">████</td></tr>
 * <tr><td>5</td><td style="color:#ffd92f">████</td></tr>
 * <tr><td>6</td><td style="color:#e5c494">████</td></tr>
 * <tr><td>7</td><td style="color:#b3b3b3">████</td></tr>
 * </table>
 */
// The HTML table above was generated using: ./scripts/scale-colours.sh html
static const std::array colours = {
    RGB{102, 194, 165},
    RGB{252, 141, 98},
    RGB{141, 160, 203},
    RGB{231, 138, 195},
    RGB{166, 216, 84},
    RGB{255, 217, 47},
    RGB{229, 196, 148},
    RGB{179, 179, 179},
};

/** Return the octree scale corresponding to \p octant_size in voxels. */
constexpr Scale from_size(const int octant_size);

/** Return the octant size in voxels corresponding to \p octant_scale. */
constexpr int to_size(const Scale octant_scale);

/** Return the color from se::scale::colours that corresponds to \p octant_scale. If \p octant_scale
 * is greater than the number of colours in se::scale::colours then the last colour will be
 * returned.
 */
constexpr RGB to_colour(const Scale octant_scale);

} // namespace scale

} // namespace se

#include "impl/scale_impl.hpp"

#endif // SE_COMMON_SCALE_HPP
