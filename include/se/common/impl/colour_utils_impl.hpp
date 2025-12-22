/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COLOUR_UTILS_IMPL_HPP
#define SE_COLOUR_UTILS_IMPL_HPP

namespace se {

namespace colour {

RGB blend(const RGB a, const RGB b, const float alpha)
{
    assert(alpha >= 0.0f);
    assert(alpha <= 1.0f);
    const float inv_alpha = 1.0f - alpha;
    return {std::uint8_t(alpha * a.r + inv_alpha * b.r),
            std::uint8_t(alpha * a.g + inv_alpha * b.g),
            std::uint8_t(alpha * a.b + inv_alpha * b.b)};
}

RGBA blend(const RGBA a, const RGBA b, const float alpha)
{
    assert(alpha >= 0.0f);
    assert(alpha <= 1.0f);
    const float inv_alpha = 1.0f - alpha;
    return {std::uint8_t(alpha * a.r + inv_alpha * b.r),
            std::uint8_t(alpha * a.g + inv_alpha * b.g),
            std::uint8_t(alpha * a.b + inv_alpha * b.b),
            std::uint8_t(alpha * a.a + inv_alpha * b.a)};
}

} // namespace colour



RGB scale_colour(const Scale scale)
{
    assert(scale >= 0);
    if (static_cast<size_t>(scale) < colours::scale.size()) {
        return colours::scale[scale];
    }
    return colours::scale.back();
}

} // namespace se

#endif // SE_COLOUR_UTILS_IMPL_HPP
