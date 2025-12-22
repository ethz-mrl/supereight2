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

/** The coordinates of the field sample in each voxel. Relative to the voxel's vertex that's closest
 * to the origin. All coordinates must be in the interval [0, 1).
 */
static inline const Eigen::Vector3f g_sample_offset = Eigen::Vector3f::Constant(0.5f);

// Representation enums
enum class Field { TSDF, Occupancy };
enum class Colour { Off, On };
enum class Id { Off, On };

// Other enums
enum class Res { Single, Multi };
enum class Safe { On = true, Off = false }; // Switch between Safe and Sorry

/**
 *  \brief The enum classes to define the sorting templates
 */
enum class Sort { SmallToLarge, LargeToSmall };

} // namespace se

#endif // SE_SETUP_UTIL_HPP
