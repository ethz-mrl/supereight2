/*
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/image/util.hpp"

#include <cassert>
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

void depth_to_rgba(const Image<float>& depth,
                   Image<RGBA>& rgba,
                   const float min_depth,
                   const float max_depth)
{
    assert(depth.width() == rgba.width());
    assert(depth.height() == rgba.height());
    assert(min_depth >= 0.0f);
    assert(max_depth > min_depth);
    const float inv_depth_range = 1.0f / (max_depth - min_depth);
#pragma omp parallel for
    for (size_t i = 0; i < depth.size(); i++) {
        const float d = depth[i];
        if (d <= 0.0f || std::isnan(d)) {
            rgba[i] = {0x00, 0x00, 0x00}; // Black
        }
        else if (d < min_depth) {
            rgba[i] = {0x80, 0x80, 0x80}; // Gray
        }
        else if (d > max_depth) {
            rgba[i] = {0xFF, 0xFF, 0xFF}; // White
        }
        else {
            const float normalized_depth = (d - min_depth) * inv_depth_range;
            const tinycolormap::Color c =
                tinycolormap::GetColor(1.0f - normalized_depth, tinycolormap::ColormapType::Heat);
            rgba[i] = {c.ri(), c.gi(), c.bi()};
        }
    }
}

} // namespace image
} // namespace se
