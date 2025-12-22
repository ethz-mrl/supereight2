/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include <cassert>
#include <memory>
#include <se/common/colour_utils.hpp>

namespace se {

template<typename T>
class Image {
    public:
    Image(const unsigned w, const unsigned h) :
            width_(w), height_(h), owned_data_(new T[w * h]), data_ptr_(owned_data_.get())
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& value) : Image(w, h)
    {
        std::fill(data(), data() + size(), value);
    }

    Image(const unsigned w, const unsigned h, T* raw_buffer) :
            width_(w), height_(h), data_ptr_(raw_buffer)
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const Image& other) = delete;

    Image(Image&& other) = default;

    Image& operator=(const Image& other) = delete;

    Image& operator=(Image&& other) = default;

    T& operator[](std::size_t idx)
    {
        return data_ptr_[idx];
    }

    const T& operator[](std::size_t idx) const
    {
        return data_ptr_[idx];
    }

    T& operator()(const int x, const int y)
    {
        return data_ptr_[x + y * width_];
    }

    const T& operator()(const int x, const int y) const
    {
        return data_ptr_[x + y * width_];
    }

    std::size_t size() const
    {
        return width_ * height_;
    }

    int width() const
    {
        return width_;
    }

    int height() const
    {
        return height_;
    }

    const T* data() const
    {
        return data_ptr_;
    }

    T* data()
    {
        return data_ptr_;
    }

    Image clone() const
    {
        if (owned_data_) {
            // Perform a deep copy of the owned data.
            Image image_copy(width(), height());
            std::copy(data(), data() + size(), image_copy.data());
            return image_copy;
        }
        else {
            // Wrap the non-owned data. The constructor accepting a non-const pointer to non-owned
            // data so it's always save to cast away the const of non-owned data().
            return Image(width(), height(), const_cast<T*>(data()));
        }
    }

    private:
    int width_;
    int height_;
    std::unique_ptr<T[]> owned_data_;
    T* data_ptr_;
};



namespace image {

/** Remap \p input to \p output by using a \p map which contains and index into \p input for each
 * element of \p output.
 */
template<typename T>
void remap(const Image<T>& input, Image<T>& output, const Image<size_t>& map);

void rgb_to_rgba(const Image<RGB>& rgb, Image<RGBA>& rgba);

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

#include "impl/image_impl.hpp"

#endif // SE_IMAGE_HPP
