/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DENSE_POOLING_IMAGE
#define SE_DENSE_POOLING_IMAGE

#include <iostream>
#include <se/common/timings.hpp>
#include <se/sensor/sensor.hpp>

namespace se {

using Value = float;
using Status = int;

struct Pixel {
    // STATUS Crossing: Voxel image intersection
    enum StatusCrossing { Inside = 0, Crossing = 1, Outside = 2 };

    // STATUS Known: Voxel content
    enum StatusKnown { Known = 0, PartKnown = 1, Unknown = 2 };

    Value min;
    Value max;
    StatusCrossing crossing;
    StatusKnown known;

    static Pixel knownPixel()
    {
        return Pixel{std::numeric_limits<Value>::max(),
                     std::numeric_limits<Value>::min(),
                     StatusCrossing::Inside,
                     StatusKnown::Known};
    };

    static Pixel unknownPixel()
    {
        return Pixel{std::numeric_limits<Value>::max(),
                     std::numeric_limits<Value>::min(),
                     StatusCrossing::Inside,
                     StatusKnown::Unknown};
    };

    static Pixel crossingKnownPixel()
    {
        return Pixel{std::numeric_limits<Value>::max(),
                     std::numeric_limits<Value>::min(),
                     StatusCrossing::Crossing,
                     StatusKnown::Known};
    };

    static Pixel crossingPartKnownPixel()
    {
        return Pixel{std::numeric_limits<Value>::max(),
                     std::numeric_limits<Value>::min(),
                     StatusCrossing::Crossing,
                     StatusKnown::PartKnown};
    };

    static Pixel crossingUnknownPixel()
    {
        return Pixel{std::numeric_limits<Value>::max(),
                     std::numeric_limits<Value>::min(),
                     StatusCrossing::Crossing,
                     StatusKnown::Unknown};
    };

    static Pixel outsidePixelBatch()
    {
        return Pixel{0, 0, StatusCrossing::Outside, StatusKnown::Unknown};
    };
};

template<typename SensorImplType>
class DensePoolingImage {
    public:
    using Img = std::vector<Pixel>;
    using Imgs = std::vector<Img>;

    DensePoolingImage(const se::Image<float>& depth_image);

    bool inImage(const int u, const int v) const;
    Pixel conservativeQuery(const Eigen::Vector2i& bb_min, const Eigen::Vector2i& bb_max) const;
    Pixel poolBoundingBox(int u_min, int u_max, int v_min, int v_max) const;

    int width() const
    {
        return image_width_;
    };
    int height() const
    {
        return image_height_;
    };
    Value maxValue() const
    {
        return image_max_value_;
    };
    int maxLevel() const
    {
        return image_max_level_;
    }

    private:
    int image_max_level_;
    int image_width_;
    int image_height_;
    Imgs pooling_image_;
    Value image_max_value_;
};



} // namespace se

#include "impl/dense_pooling_image_impl.hpp"

#endif // SE_DENSE_POOLING_IMAGE
