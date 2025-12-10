/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_TYPE_UTIL_HPP
#define SE_TYPE_UTIL_HPP

#include <Eigen/Dense>
#include <se/common/id.hpp>

namespace se {

/** \defgroup OctreeKey
 * Types used for compact encoding of octant coordinates and size. se::key_t is the main type, while
 * se::code_t and se::scale_t describe components of se::key_t. There's usually no reason to work
 * with anything other than se::key_t unless you're modifying the internals of supereight2.
 * @{
 */

/** Compactly encodes the 3D coordinates and size of an se::OctantBase.
 *
 * The 3D coordinates and scale are stored in the 64 bits like this:
 * \code{text}
 *      MSB                                                          LSB
 * Bit 63 61                                                      54   0
 *      ↓ ↓                                                       ↓↓   ↓
 *      --zyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxsssss
 *      ├┘└───────────────────────────┬───────────────────────────┘└─┬─┘
 *      Unused                 3D coordinates                      Scale
 *                            (code_t 57 LSBs)                (scale_t 5 LSBs)
 * \endcode
 *
 * See se::code_t and se::scale_t for details on the individual components.
 *
 * \note This is how the number of bits allocated for coordinates and scale were computed. Given
 * that each scale requires 3 bits to represent in a Morton code and we've got 64 bits available:
 * 1. `floor(64 / 3) = 21`, we can store up to 21 scales. `64 - (3 * 21) = 1`, 1 bit is not enough
 *    to represent 21 numbers.
 * 2. If we store 20 scales, `64 - (3 * 20) = 4`, 4 bits are not enough to represent 20 numbers.
 * 3. If we store 19 scales, `64 - (3 * 19) = 7`, 7 bits are enough to represent 19 numbers.
 */
typedef uint64_t key_t;

/** 3D coordinates encoded as a 57-bit Morton code. Each coordinate can take values in the range
 * [0, 524287], since 19 bits (57/3) are used for each coordinate. For e.g. a 2 mm voxel resolution
 * this allows mapping a 1×1×1 km region.
 *
 * The 3D coordinates are stored interleaved in the 64 bits like this:
 * \code{text}
 *     MSB                                                          LSB
 * Bit 63     56                                                      0
 *     ↓      ↓                                                       ↓
 *     -------zyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyxzyx
 *     └──┬──┘└┬┘                                                   └┬┘
 *     Unused  x,y,z coordinate MSBs             x,y,z coordinate LSBs
 * \endcode
 *
 * For more on Morton codes see here: https://en.wikipedia.org/wiki/Z-order_curve
 */
typedef uint64_t code_t;

/** The 5-bit scale stored in an se::key_t. See se::Scale for an explanation of scale. Can take
 * values in the range [0, 31] but only values in the range [0, 18] are used. It is stored in the 5
 * LSBs of the uint64_t.
 */
typedef uint64_t scale_t;

/** @} */



/** A linear voxel or child index. */
typedef unsigned int idx_t;

/** The field stored in the octree. E.g. TSDF or occupancy. */
typedef float field_t;

/** A 3D field gradient. */
typedef Eigen::Matrix<field_t, 3, 1> field_vec_t;

/** The weight associated with the field. */
typedef se::field_t weight_t;

/** A timestamp. Currently stores a frame number. -1 indicates an uninitialized/invalid timestamp. */
typedef int timestamp_t;

/** The color stored in the octree. */
typedef RGB colour_t;

} // namespace se

#endif // SE_TYPE_UTIL_HPP
