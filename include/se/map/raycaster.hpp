/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Nils Funk
 * SPDX-FileCopyrightText: 2020-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#include <optional>
#include <se/common/colour_utils.hpp>
#include <se/common/eigen_utils.hpp>
#include <se/common/math_util.hpp>
#include <se/common/rgb.hpp>
#include <se/common/rgba.hpp>
#include <se/image/image.hpp>
#include <se/map/octree/visitor.hpp>
#include <se/map/octree/voxel_block_ray_iterator.hpp>
#include <se/map/preprocessor.hpp>



namespace se {
namespace raycaster {

void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud,
                           const bool is_lhc = false);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const typename MapT::OctreeType& octree,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        const float t_near,
        const float t_far,
        const float mu,
        const float step,
        const float largestep);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const typename MapT::OctreeType& octree,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        const float t_near,
        const float t_far,
        const float mu,
        const float step,
        const float largestep);

template<typename MapT, typename SensorT>
void raycast_volume(const MapT& map,
                    const SensorT& sensor,
                    const Eigen::Isometry3f& T_WS,
                    se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                    se::Image<Eigen::Vector3f>& surface_normals_W,
                    se::Image<int8_t>& surface_scale,
                    se::Image<colour_t>* surface_colour = nullptr,
                    se::Image<id_t>* surface_id = nullptr);

/** Render the surface represented by \p surface_points_W and \p surface_normals_W into \p render.
 * The colour of each point is returned by the functor \p get_diffuse_colour which must have the
 * following prototype:
 * \code
 * se::RGB get_diffuse_colour(const size_t pixel_index);
 * \endcode
 * where \p pixel_index is a linear index into \p surface_points_W.
 *
 * The scene is lit by a point light located at \p light_source_W and an ambient light with colour
 * \p ambient_light.
 *
 * Uses [Gouraud shading](https://en.wikipedia.org/wiki/Gouraud_shading#Description) with a
 * simplified
 * [Phong reflection model](https://en.wikipedia.org/wiki/Phong_reflection_model#Description)
 * containing only ambient and diffuse components (no specular component.
 */
template<typename GetDiffuseColourF>
void render_volume(se::Image<RGBA>& render,
                   const se::Image<Eigen::Vector3f>& surface_points_W,
                   const se::Image<Eigen::Vector3f>& surface_normals_W,
                   const GetDiffuseColourF get_diffuse_colour,
                   const Eigen::Vector3f& light_source_W = Eigen::Vector3f::Zero(),
                   const RGB ambient_light = RGB{0x1A, 0x1A, 0x1A});

/** Render the surface represented by \p surface_points_W and \p surface_normals_W into \p render,
 * coloured using the scale from \p surface_scale. Lighting is as in se::raycaster::render_volume().
 */
void render_volume_scale(se::Image<RGBA>& render,
                         const se::Image<Eigen::Vector3f>& surface_points_W,
                         const se::Image<Eigen::Vector3f>& surface_normals_W,
                         const se::Image<int8_t>& surface_scale,
                         const Eigen::Vector3f& light_source_W = Eigen::Vector3f::Zero(),
                         const RGB ambient_light = RGB{0x1A, 0x1A, 0x1A});

/** Render the surface represented by \p surface_points_W and \p surface_normals_W into \p render,
 * coloured using the colours from \p surface_colour. Lighting is as in
 * se::raycaster::render_volume().
 */
void render_volume_colour(se::Image<RGBA>& render,
                          const se::Image<Eigen::Vector3f>& surface_points_W,
                          const se::Image<Eigen::Vector3f>& surface_normals_W,
                          const se::Image<RGB>& surface_colour,
                          const Eigen::Vector3f& light_source_W = Eigen::Vector3f::Zero(),
                          const RGB ambient_light = RGB{0x1A, 0x1A, 0x1A});

/** Given a map \p map and a depth image \p depth and the sensor characteristics
 * as provided by \p sensor and the pose in the map frame \p T_WC,
 * this function extracts the ids for each pixel in the depth image.
 */
template<typename MapT, typename SensorT>
Image<id_t> lookup_ids(const MapT& map,
                       const Image<float>& depth,
                       const SensorT& sensor,
                       const Eigen::Isometry3f& T_WC);

} // namespace raycaster
} // namespace se

#include "impl/raycaster_impl.hpp"

#endif // SE_RAYCASTER_HPP
