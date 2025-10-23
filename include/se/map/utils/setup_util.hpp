/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SETUP_UTIL_HPP
#define SE_SETUP_UTIL_HPP

#include <Eigen/Core>

namespace se {

static inline const Eigen::Vector3f sample_offset_frac = Eigen::Vector3f::Constant(0.5f);

// Representation enums
enum class Field { TSDF, Occupancy };
enum class Colour { Off, On };
enum class Id { Off, On };

// Other enums
enum class Res { Single, Multi };
enum class Integ { Simple, LiDAR, PinholeCamera };
enum class Safe { On = true, Off = false }; // Switch between Safe and Sorry

enum class AllocMeth { Raycasting, VoxelCarving }; // Allocation method
enum class Rep { Surface, Freespace };             // Map representation

/**
 *  \brief The enum classes to define the sorting templates
 */
enum class Sort { SmallToLarge, LargeToSmall };

} // namespace se

#endif // SE_SETUP_UTIL_HPP
