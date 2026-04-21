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
    Value min;
    Value max;

    // STATUS Crossing: Voxel image intersection
    enum statusCrossing { inside = 0, crossing = 1, outside = 2 };

    // STATUS Known: Voxel content
    enum statusKnown { known = 0, part_known = 1, unknown = 2 };

    statusCrossing status_crossing;
    statusKnown status_known;

    Pixel(){};

    Pixel(Value min, Value max, statusCrossing status_crossing, statusKnown status_known) :
            min(min), max(max), status_crossing(status_crossing), status_known(status_known){};

    static Pixel knownPixel()
    {
        Pixel knownPixel(std::numeric_limits<Value>::max(),
                         std::numeric_limits<Value>::min(),
                         statusCrossing::inside,
                         statusKnown::known);
        return knownPixel;
    };

    static Pixel unknownPixel()
    {
        Pixel unknownPixel(std::numeric_limits<Value>::max(),
                           std::numeric_limits<Value>::min(),
                           statusCrossing::inside,
                           statusKnown::unknown);
        return unknownPixel;
    };

    static Pixel crossingKnownPixel()
    {
        Pixel crossingPixel(std::numeric_limits<Value>::max(),
                            std::numeric_limits<Value>::min(),
                            statusCrossing::crossing,
                            statusKnown::known);
        return crossingPixel;
    };

    static Pixel crossingPartKnownPixel()
    {
        Pixel crossingPixel(std::numeric_limits<Value>::max(),
                            std::numeric_limits<Value>::min(),
                            statusCrossing::crossing,
                            statusKnown::part_known);
        return crossingPixel;
    };

    static Pixel crossingUnknownPixel()
    {
        Pixel crossingPixel(std::numeric_limits<Value>::max(),
                            std::numeric_limits<Value>::min(),
                            statusCrossing::crossing,
                            statusKnown::unknown);
        return crossingPixel;
    };

    static Pixel outsidePixelBatch()
    {
        Pixel outsidePixel(0, 0, statusCrossing::outside, statusKnown::unknown);
        return outsidePixel;
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
