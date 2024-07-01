/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/colour_utils.hpp"

#include <se/external/tinycolormap.hpp>

void se::depth_to_rgba(se::RGBA* depth_RGBA_image_data,
                       const float* depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const float min_depth,
                       const float max_depth)
{
    const float inv_depth_range = 1.0f / (max_depth - min_depth);
    const size_t num_pixels = size_t(depth_image_res.x()) * size_t(depth_image_res.y());
#pragma omp parallel for
    for (size_t i = 0; i < num_pixels; i++) {
        const float depth = depth_image_data[i];
        if (depth <= 0.0f || std::isnan(depth)) {
            depth_RGBA_image_data[i] = {0x00, 0x00, 0x00}; // Black
        }
        else if (depth < min_depth) {
            depth_RGBA_image_data[i] = {0x80, 0x80, 0x80}; // Gray
        }
        else if (depth > max_depth) {
            depth_RGBA_image_data[i] = {0xFF, 0xFF, 0xFF}; // White
        }
        else {
            const float normalized_depth = (depth - min_depth) * inv_depth_range;
            const tinycolormap::Color c =
                tinycolormap::GetColor(1.0f - normalized_depth, tinycolormap::ColormapType::Heat);
            depth_RGBA_image_data[i] = {c.ri(), c.gi(), c.bi()};
        }
    }
}
