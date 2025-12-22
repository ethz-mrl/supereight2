/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SETUP_UTIL_HPP
#define SE_SETUP_UTIL_HPP

#include <Eigen/Core>

namespace se {

/** The coordinates of the field sample in each voxel. Relative to the voxel's vertex that's closest
 * to the origin. All coordinates must be in the interval [0, 1).
 */
static inline const Eigen::Vector3f g_sample_offset = Eigen::Vector3f::Constant(0.5f);



/** The type of field stored in the voxels. */
enum class Field {
    /** Truncated signed distance function. */
    TSDF,
    /** Occupancy probability expressed in log-odds. */
    Occupancy,
};

/** Whether the voxels contain colour data of type se::colour_t. */
enum class Colour { Off, On };

/** Whether the voxels contain identifier data of type se::id_t. */
enum class Id { Off, On };



/** Distinguish between single- and multi-resolution se::Field::TSDF representations. Only
 * se::Res::Multi is valid for se::Field::Occupancy. */
enum class Res {
    /** A classic TSDF representation. Stores data only in the octree leaves. */
    Single,
    /** An adaptive-resolution TSDF representation. Stores data at various scales (see se::Scale) of
     * an se::Block.
     */
    Multi,
};

/** Whether to enable certain safety and bounds checks. */
enum class Safe { On = true, Off = false };

/** The sort order. */
enum class Sort {
    /** Ascending. */
    SmallToLarge,
    /** Descending. */
    LargeToSmall,
};

} // namespace se

#endif // SE_SETUP_UTIL_HPP
