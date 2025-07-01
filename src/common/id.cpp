/*
 * SPDX-FileCopyrightText: 2024-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/common/id.hpp>

namespace se {

RGB id_colour(const id_t id)
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
