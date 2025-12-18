/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "se/map/preprocessor.hpp"

#include <se/common/eigen_utils.hpp>
#include <se/common/math_util.hpp>

namespace se {
namespace preprocessor {

Image<size_t> downsample_depth(const Image<float>& input_depth_img, Image<float>& output_depth_img)
{
    const int w_in = input_depth_img.width();
    const int w_out = output_depth_img.width();
    const int h_out = output_depth_img.height();

    assert((w_in >= w_out) && "The input width isn't smaller than the output width");
    assert((input_depth_img.height() >= h_out)
           && "The input height isn't smaller than the output height");
    assert((w_in % w_out == 0) && "The input width is an integer multiple of the output width");
    assert((input_depth_img.height() % h_out == 0)
           && "The input height is an integer multiple of the output height");
    assert((w_in / w_out == input_depth_img.height() / h_out)
           && "The input and output image aspect ratios are the same");

    struct Pixel {
        int x;
        int y;
        float depth;
    };

    Image<size_t> map(w_out, h_out);
    const int ratio = w_in / w_out;
#pragma omp parallel for
    for (int y_out = 0; y_out < h_out; y_out++) {
        std::vector<Pixel> region;
        region.reserve(ratio * ratio);
        for (int x_out = 0; x_out < w_out; x_out++) {
            // Iterate over the region of the input image that will be aggregated into this pixel of
            // the output image and keep track of pixels containing valid depth values.
            const int x_in_base = x_out * ratio;
            const int y_in_base = y_out * ratio;
            for (int y_region = 0; y_region < ratio; y_region++) {
                for (int x_region = 0; x_region < ratio; x_region++) {
                    const int x_in = x_in_base + x_region;
                    const int y_in = y_in_base + y_region;
                    const float depth_value = input_depth_img(x_in, y_in);
                    // Only consider positive, non-NaN values.
                    if (depth_value >= 1e-5f) {
                        region.push_back({x_in, y_in, depth_value});
                    }
                }
            }
            if (region.empty()) {
                // All depth values in the region were invalid, set the depth of the output image to
                // invalid and arbitrarily map to the first pixel in the region.
                output_depth_img(x_out, y_out) = 0.0f;
                map(x_out, y_out) = x_in_base + y_in_base * w_in;
            }
            else {
                // Sort the region pixels based on depth.
                std::sort(region.begin(), region.end(), [](const auto& a, const auto& b) {
                    return a.depth < b.depth;
                });
                // Find the region pixel containing the median depth. Don't average the midpoints
                // when region contains an even number of elements to avoid introducing unobserved
                // depth values. Get the first of the two middle elements instead.
                const Pixel value = region[region.size() / 2];
                output_depth_img(x_out, y_out) = value.depth;
                map(x_out, y_out) = value.x + value.y * w_in;
            }
            region.clear();
        }
    }
    return map;
}



void point_cloud_to_depth(se::Image<float>& depth_image,
                          const se::Image<Eigen::Vector3f>& point_cloud_X,
                          const Eigen::Isometry3f& T_CX)
{
#pragma omp parallel for
    for (int y = 0; y < depth_image.height(); y++) {
        for (int x = 0; x < depth_image.width(); x++) {
            depth_image(x, y) = (T_CX * point_cloud_X(x, y)).z();
        }
    }
}



// Explicit template instantiation
template void point_cloud_to_normal<true>(se::Image<Eigen::Vector3f>& normals,
                                          const se::Image<Eigen::Vector3f>& point_cloud);

template void point_cloud_to_normal<false>(se::Image<Eigen::Vector3f>& normals,
                                           const se::Image<Eigen::Vector3f>& point_cloud);

template<bool NegY>
void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud)
{
    const int width = point_cloud.width();
    const int height = point_cloud.height();
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const Eigen::Vector3f point = point_cloud[x + width * y];
            if (point.z() == 0.f) {
                normals[x + y * width] = math::g_invalid_normal;
                continue;
            }

            const Eigen::Vector2i p_left = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
            const Eigen::Vector2i p_right = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

            // Swapped to match the left-handed coordinate system of ICL-NUIM
            Eigen::Vector2i p_up, p_down;
            if (NegY) {
                p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }
            else {
                p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }

            const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
            const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
            const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
            const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

            if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
                normals[x + y * width] = math::g_invalid_normal;
                continue;
            }
            const Eigen::Vector3f dv_x = right - left;
            const Eigen::Vector3f dv_y = up - down;
            normals[x + y * width] = dv_x.cross(dv_y).normalized();
        }
    }
}



void half_sample_robust_image(se::Image<float>& output_image,
                              const se::Image<float>& input_image,
                              const float e_d,
                              const int radius)
{
    if ((input_image.width() / 2 != output_image.width())
        || (input_image.height() / 2 != output_image.height())) {
        output_image = se::Image<float>(input_image.width() / 2, input_image.height() / 2);
    }

#pragma omp parallel for
    for (int y = 0; y < output_image.height(); y++) {
        for (int x = 0; x < output_image.width(); x++) {
            const Eigen::Vector2i out_pixel = Eigen::Vector2i(x, y);
            const Eigen::Vector2i in_pixel = 2 * out_pixel;

            float pixel_count = 0.0f;
            float pixel_value_sum = 0.0f;
            const float in_pixel_value =
                input_image[in_pixel.x() + in_pixel.y() * input_image.width()];
            for (int i = -radius + 1; i <= radius; ++i) {
                for (int j = -radius + 1; j <= radius; ++j) {
                    Eigen::Vector2i in_pixel_tmp = in_pixel + Eigen::Vector2i(j, i);
                    se::eigen::clamp(in_pixel_tmp,
                                     Eigen::Vector2i::Zero(),
                                     Eigen::Vector2i(2 * output_image.width() - 1,
                                                     2 * output_image.height() - 1));
                    const float in_pixel_value_tmp =
                        input_image[in_pixel_tmp.x() + in_pixel_tmp.y() * input_image.width()];
                    if (fabsf(in_pixel_value_tmp - in_pixel_value) < e_d) {
                        pixel_count += 1.0f;
                        pixel_value_sum += in_pixel_value_tmp;
                    }
                }
            }
            output_image[out_pixel.x() + out_pixel.y() * output_image.width()] =
                pixel_value_sum / pixel_count;
        }
    }
}

} // namespace preprocessor
} // namespace se
