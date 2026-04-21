/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/raycaster.hpp"

namespace se {
namespace raycaster {



void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud,
                           const bool /* is_lhc */)
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
            if (false) {
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
        } // x
    }     // y
}



void render_volume_scale(se::Image<RGBA>& render,
                         const se::Image<Eigen::Vector3f>& surface_points_W,
                         const se::Image<Eigen::Vector3f>& surface_normals_W,
                         const se::Image<int8_t>& surface_scale,
                         const Eigen::Vector3f& light_source_W,
                         const RGB ambient_light)
{
    assert(render.width() == surface_scale.width());
    assert(render.height() == surface_scale.height());
    return render_volume(
        render,
        surface_points_W,
        surface_normals_W,
        [&surface_scale](const size_t pixel_idx) {
            return scale::to_colour(surface_scale[pixel_idx]);
        },
        light_source_W,
        ambient_light);
}



void render_volume_colour(se::Image<RGBA>& render,
                          const se::Image<Eigen::Vector3f>& surface_points_W,
                          const se::Image<Eigen::Vector3f>& surface_normals_W,
                          const se::Image<RGB>& surface_colour,
                          const Eigen::Vector3f& light_source_W,
                          const RGB ambient_light)
{
    assert(render.width() == surface_colour.width());
    assert(render.height() == surface_colour.height());
    return render_volume(
        render,
        surface_points_W,
        surface_normals_W,
        [&surface_colour](const size_t pixel_idx) { return surface_colour[pixel_idx]; },
        light_source_W,
        ambient_light);
}

} // namespace raycaster
} // namespace se
