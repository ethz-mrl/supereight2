/*
 * SPDX-FileCopyrightText: 2024-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_ID_HPP
#define SE_COMMON_ID_HPP

#include <se/common/rgb.hpp>

namespace se {

/** An identifier associated to some data. It can be used as a semantic class, object instance ID or
 * for any kind of association of external data to octree voxels. */
typedef uint16_t id_t;

/** Indicates the absence of an identifier. */
inline constexpr id_t g_no_id = 0;

/** Indicates an unmapped region, which is distinct from a region without an identifier. Underflow
 * is well-defined for unsigned integers. */
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

#endif // SE_COMMON_ID_HPP
