/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_TYPE_UTIL_HPP
#define SE_TYPE_UTIL_HPP

#include <Eigen/StdVector>
#include <se/common/rgb.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

namespace se {

/**
 * \brief key = 1 bit buffer + 57 bits of morton code + 6 bits of scale information
 *        The maxium scale is limited by 57 / 3 = 19 scales
 *
 *
 * \note uint64_t has 64 bits
 *       We'll use the key to store both, the Morton code and scale information
 *       In 3D three bits are required to encode the Morton code at each scale.
 *       21 scales -> 3 bits * 21 = 63 bits; 64 bits - 63 bits = 1 bit  to encode unsigned int up to 21 [not possible]
 *       20 scales -> 3 bits * 20 = 60 bits; 64 bits - 60 bits = 4 bits to encode unsigned int up to 20 [not possible]
 *       19 scales -> 3 bits * 19 = 57 bits; 64 bits - 57 bits = 7 bits to encode unsigend int up to 19 [possible]
 *
 *       The tree cannot allocate any depth further than 19 allowing a map size = 524288 * map resolution
 *       That is a maximum map size of 1 x 1 x 1 km^3 at 2 mm resolution
 */
typedef uint64_t key_t;   ///< The type of the Key i.e. code | scale
typedef uint64_t code_t;  ///< The type of the Morton code
typedef uint64_t scale_t; ///< The type of the scale in the morton code

typedef unsigned int idx_t; ///< Child or voxel index type

typedef float field_t; ///< The type of the stored field (e.g. TSDF, ESDF or occupancy)

typedef Eigen::Matrix<field_t, 3, 1> field_vec_t;

typedef se::field_t weight_t; ///< The type of the field type weight

typedef int timestamp_t; ///< The type of the time stamp

typedef RGB colour_t; ///< The type of the colour

/** The type used to represent identifiers. */
typedef uint16_t id_t;
/** Indicates the absence of an identifier. */
inline constexpr id_t g_no_id = 0;
/** Used to distinguish a region that's unmapped from a region without an identifier. Underflow is
 * well-defined for unsigned integers. */
inline constexpr id_t g_not_mapped = -1;


/** Function that generates a unique color based on the id received
 * \param[in] id The segment id
 * \param[out] The RGB color
 */
static inline RGB id_colour(const id_t id)
{
    switch (id) {
    case g_not_mapped:
        return {0x00, 0x00, 0x00};
    case g_no_id:
        return {0xFF, 0xFF, 0xFF};
    default: {
        // Inspired from the following and naively modified for 16-bit integers.
        // https://stackoverflow.com/questions/664014/what-integer-hash-function-are-good-that-accepts-an-integer-hash-key/12996028#12996028
        const uint8_t r = ((id >> 8) ^ id) * 0x45d9f3b;
        const uint8_t g = ((id >> 8) ^ r) * 0x45d9f3b;
        const uint8_t b = ((id >> 8) ^ g) * 0x45d9f3b;
        return {r, g, b};
    }
    }
}

} // namespace se

#endif // SE_TYPE_UTIL_HPP
