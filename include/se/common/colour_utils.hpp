/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COLOUR_UTILS_HPP
#define SE_COLOUR_UTILS_HPP

#include <Eigen/Core>
#include <cstdint>
#include <se/common/rgb.hpp>
#include <se/common/rgba.hpp>
#include <vector>



namespace se {
namespace colours {
/**
 * The colours used for the various integration scales.
 */
static const std::vector<RGB> scale = {{102, 194, 165},
                                       {252, 141, 98},
                                       {141, 160, 203},
                                       {231, 138, 195},
                                       {166, 216, 84},
                                       {255, 217, 47},
                                       {229, 196, 148},
                                       {179, 179, 179}};
} // namespace colours



namespace colour {

/** Blend colors \p a and \p b based on the value of \p alpha. Returns per-channel
 * `alpha * a + (1 - alpha) * b`. The value of alpha must be in the range [0, 1] inclusive.
 *
 * \note Swapping \p a and \p b while keeping the same \p alpha is not guaranteed to produce the
 * same result.
 */
static inline RGB blend(const RGB a, const RGB b, const float alpha);

/** \overload */
static inline RGBA blend(const RGBA a, const RGBA b, const float alpha);

} // namespace colour



/**
 * Convert a depth image to an RGBA image to allow visualizing it.
 * The depth image is scaled using the minimum and maximum depth values to
 * increase contrast.
 *
 * \param[in] depth_RGBA_image_data Pointer to the ouput RGBA image data.
 * \param[in] depth_image_data      Pointer to the input depth image data.
 * \param[in] depth_image_res       Resolution of the depth image in pixels
 *                                  (width and height).
 * \param[in] min_depth             The minimum possible depth value.
 * \param[in] max_depth             The maximum possible depth value.
 */
void depth_to_rgba(RGBA* depth_RGBA_image_data,
                   const float* depth_image_data,
                   const Eigen::Vector2i& depth_image_res,
                   const float min_depth,
                   const float max_depth);

/** Return the color from se::colours::scale that should be used to visualize the supplied \p scale.
 * If the scale is greater or equal to the number of colours in se::colours::scale then the last
 * colour will be returned.
 */
static inline const RGB scale_colour(const int scale);

} // namespace se

#include "impl/colour_utils_impl.hpp"

#endif // SE_COLOUR_UTILS_HPP
