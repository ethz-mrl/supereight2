/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_UTIL_HPP
#define SE_IMAGE_UTIL_HPP

#include <se/image/image.hpp>

namespace se {
/** Functions for converting and processing images of type se::Image. */
namespace image {

/** Remap \p input to \p output by using a \p map which contains and index into \p input for each
 * element of \p output.
 */
template<typename T>
void remap(const Image<T>& input, Image<T>& output, const Image<size_t>& map);

/** Convert an RGB image to RGBA, adding a fully opaque alpha channel. */
void rgb_to_rgba(const Image<RGB>& rgb, Image<RGBA>& rgba);

/** Convert an RGBA image to RGB, discarding the alpha channel. */
void rgba_to_rgb(const Image<RGBA>& rgba, Image<RGB>& rgb);

/** Write a colour visualization of the depth image \p depth into \p rgba. The depth image is scaled
 * using \p min_depth and \p max_depth to increase contrast before mapping it to colours. Invalid
 * depth values are shown in black, values smaller than \p min_depth in gray, values greater than \p
 * max_depth in white and all other values using tinycolormap::ColormapType::Heat.
 */
void depth_to_rgba(const Image<float>& depth,
                   Image<RGBA>& rgba,
                   const float min_depth = 0.0f,
                   const float max_depth = std::numeric_limits<float>::max());

} // namespace image
} // end namespace se

#endif // SE_IMAGE_UTIL_HPP
