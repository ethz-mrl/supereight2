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
        // Inspired from https://stackoverflow.com/a/12996028 and modified for 16-bit integers.
        const uint8_t r = 0x45d9f3b * ((id >> 8) ^ id);
        const uint8_t g = 0x45d9f3b * ((id >> 8) ^ r);
        const uint8_t b = 0x45d9f3b * ((id >> 8) ^ g);
        return {r, g, b};
    }
    }
}

} // namespace se
