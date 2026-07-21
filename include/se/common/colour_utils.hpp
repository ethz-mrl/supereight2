/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_COLOUR_UTILS_HPP
#define SE_COMMON_COLOUR_UTILS_HPP

#include <Eigen/Core>
#include <array>
#include <se/common/rgb.hpp>
#include <se/common/rgba.hpp>
#include <se/common/scale.hpp>



namespace se {

/** Functions for processing colours. */
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

} // namespace se

#include "impl/colour_utils_impl.hpp"

#endif // SE_COMMON_COLOUR_UTILS_HPP
