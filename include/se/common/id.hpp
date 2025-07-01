/*
 * SPDX-FileCopyrightText: 2024-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_ID_HPP
#define SE_COMMON_ID_HPP

#include <limits>
#include <se/common/rgb.hpp>

namespace se {

/** An identifier associated to some data. It can be used as a semantic class, object instance ID or
 * for any kind of association of external data to octree voxels. */
typedef uint16_t id_t;

/** Indicates the absence of an identifier. */
inline constexpr id_t g_no_id = 0;

/** Indicates an unmapped region, which is distinct from a region without an identifier. */
inline constexpr id_t g_not_mapped = std::numeric_limits<id_t>::max();



/** Return a (probably) unique colour for visualising \p id. */
RGB id_colour(const id_t id);

} // namespace se

#endif // SE_COMMON_ID_HPP
