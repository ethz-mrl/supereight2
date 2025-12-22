/*
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/image/image.hpp"

#include <se/external/tinycolormap.hpp>

namespace se {
namespace image {

void rgb_to_rgba(const Image<RGB>& rgb, Image<RGBA>& rgba)
{
    assert(rgb.width() == rgba.width());
    assert(rgb.height() == rgba.height());
#pragma omp parallel for
    for (size_t i = 0; i < rgb.size(); ++i) {
        const RGB pixel = rgb[i];
        rgba[i] = RGBA{pixel.r, pixel.g, pixel.b, 0xFF};
    }
}



void rgba_to_rgb(const Image<RGBA>& rgba, Image<RGB>& rgb)
{
    assert(rgba.width() == rgb.width());
    assert(rgba.height() == rgb.height());
#pragma omp parallel for
    for (size_t i = 0; i < rgba.size(); ++i) {
        const RGBA pixel = rgba[i];
        rgb[i] = RGB{pixel.r, pixel.g, pixel.b};
    }
}

void depth_to_rgba(RGBA* depth_RGBA_image_data,
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

} // namespace image
} // namespace se
