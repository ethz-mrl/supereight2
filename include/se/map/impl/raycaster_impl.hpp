/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCASTER_IMPL_HPP
#define SE_RAYCASTER_IMPL_HPP

namespace se {
namespace raycaster {



/**
 * \brief Finds the first valid point along a ray starting from (ray_origin_W + t * ray_dir_W). Returns false if no
 * valid point can be found before the maximum travelled distance is reached.
 *
 * \param[in]     map                The reference to the map
 * \param[in]     ray_origin_W       The origin of the ray in world frame in [m]
 * \param[in]     ray_dir_W          The direction of the ray in world frame
 * \param[in]     step_size          The size of a step per iteration [m]
 * \param[in]     t_max              The travel distance limit along the ray [m]
 * \param[in,out] t                  [in]  The initial travelled distance along the ray in [m],
 *                                   [out] The travelled distance along the ray in [m] at the first valid point
 * \param[out]    point_W            First valid point along the ray starting from the input t
 *
 * \return True if valid point could be found before reaching t_max. False otherwise.
 */
template<typename MapT>
inline std::optional<se::field_t> find_valid_point(const MapT& map,
                                                   const Eigen::Vector3f& ray_origin_W,
                                                   const Eigen::Vector3f& ray_dir_W,
                                                   const float step_size,
                                                   const float t_max,
                                                   float& t,
                                                   Eigen::Vector3f& point_W)
{
    std::optional<se::field_t> value = {};
    typename MapT::DataType peek_data;

    Eigen::Vector3f ray_pos_W = ray_origin_W + t * ray_dir_W;
    if (map.contains(ray_pos_W)) {
        peek_data = map.getData(ray_pos_W);
        if (is_valid(peek_data)) {
            value = map.interpField(ray_pos_W);
        }
    }

    while (!value) {
        t += step_size;
        if (t > t_max) {
            return value;
        }

        ray_pos_W = ray_origin_W + t * ray_dir_W;
        if (map.contains(ray_pos_W)) {
            peek_data = map.getData(ray_pos_W);
            if (is_valid(peek_data)) {
                value = map.interpField(ray_pos_W);
            }
        }
    }

    point_W = ray_pos_W;
    return value;
}



/**
 * \brief Compute the distance t in [m] travelled along the ray from the origin until the ray intersecs with the map.
 * Can be summariesed in 3 cases:
 * 1. Origin is inside the map                                      -> Return 0/valid
 * 2. Origin is outside the map and ray intersects the map          -> Return t/valid
 * 3. Origin is outside the map and ray will not intersect the map  -> Return 0/invalid
 *
 * \param[in]  map          The reference to the map
 * \param[in]  ray_pos_W    The origin of the ray in world frame in [m]
 * \param[in]  ray_dir_W    The direction of the ray in world frame
 * \param[in]  t_far        maximum travel distance along the ray in [m]
 * \param[out] is_valid     Bool indicating if the map intersection is valid
 *
 * \return see above
 */
template<typename MapT>
inline float compute_map_intersection(const MapT& map,
                                      const Eigen::Vector3f& ray_pos_W,
                                      const Eigen::Vector3f& ray_dir_W,
                                      const float t_far,
                                      bool& is_valid)
{
    /*
  Fast Ray-Box Intersection
  by Andrew Woo
  from "Graphics Gems", Academic Press, 1990
  */
    // Translate position and direction from world to map frame
    const Eigen::Vector3f ray_pos_M = map.getTMW() * ray_pos_W;
    const Eigen::Vector3f ray_dir_M = map.getTMW().linear() * ray_dir_W;

    const Eigen::Vector3f map_min = Eigen::Vector3f::Zero();
    const Eigen::Vector3f map_max = map.getDim();
    constexpr int num_dim = 3;
    Eigen::Vector3f hit_point = -1 * Eigen::Vector3f::Ones(); /* hit point */
    bool inside = true;
    Eigen::Vector3i quadrant;
    int which_plane;
    Eigen::Vector3f max_T;
    Eigen::Vector3f candidate_plane = Eigen::Vector3f::Zero();

    /* Find candidate planes; this loop can be avoided if
     rays cast all from the eye(assume perpsective view) */
    for (int i = 0; i < num_dim; i++) {
        if (ray_pos_M[i] < map_min[i]) {
            quadrant[i] = 1; // LEFT := 1
            candidate_plane[i] = map_min[i];
            inside = false;
        }
        else if (ray_pos_M[i] > map_max[i]) {
            quadrant[i] = 0; // RIGHT := 0
            candidate_plane[i] = map_max[i];
            inside = false;
        }
        else {
            quadrant[i] = 2; // MIDDLE := 2
        }
    }

    /* Ray origin inside bounding box */
    if (inside) {
        return 0;
    }

    /* Calculate T distances to candidate planes */
    for (int i = 0; i < num_dim; i++) {
        if (quadrant[i] != 2 && ray_dir_M[i] != 0.) // MIDDLE := 2
        {
            max_T[i] = (candidate_plane[i] - ray_pos_M[i]) / ray_dir_M[i];
        }
        else {
            max_T[i] = -1.;
        }
    }

    /* Get largest of the max_T's for final choice of intersection */
    which_plane = 0;
    for (int i = 1; i < num_dim; i++) {
        if (max_T[which_plane] < max_T[i]) {
            which_plane = i;
        }
    }

    /* Check final candidate actually inside box */
    if (max_T[which_plane] < 0.f) {
        is_valid = false;
        return 0;
    }

    for (int i = 0; i < num_dim; i++) {
        if (which_plane != i) {
            hit_point[i] = ray_pos_M[i] + max_T[which_plane] * ray_dir_M[i];
            if (hit_point[i] < map_min[i] || hit_point[i] > map_max[i]) {
                is_valid = false;
                return 0;
            }
        }
        else {
            hit_point[i] = candidate_plane[i];
        }
    }

    float t = (hit_point - ray_pos_M).norm();
    if (t_far < t) {
        is_valid = false;
        return 0;
    }
    return t;
}

/**
 * \brief Advance ray from the camera position until a voxel block with a max occupancy
 * log-odd value of at least -0.2 is reached
 *
 * \param[in]     map           The reference to the map
 * \param[in]     ray_origin_W  The origin of the ray in world frame in [m]
 * \param[in]     ray_dir_W     The ray direction in world frame
 * \param[in/out] t             The distance until the search voxel block is reached
 * \param[in/out] t_far         The distance to the far plane or until the map is surpassed
 * \param[in]     voxel_dim     Resolution of the map
 * \param[in]     max_scale     Finest scale at which to check the occupancy
 * \param[out]    is_valid      Indiactes if a voxel block was found
 */
template<typename MapT>
inline void advance_ray(const MapT& map,
                        const typename MapT::OctreeType& octree,
                        const Eigen::Vector3f& ray_origin_W,
                        const Eigen::Vector3f& ray_dir_W,
                        float& t,
                        float& t_far,
                        const int max_scale,
                        bool& is_valid)
{
    const float voxel_dim = map.getRes(); // voxel_dim   := [m / voxel];
    int scale = max_scale;                // Initialize scale
    // Additional distance travelled in [voxel]
    float v_add = 0;                   // TODO: I'll have to re-evaluate this value.
    const float v = 1 / voxel_dim * t; // t in voxel coordinates
    //float v_near  = 1 / voxel_dim * t_near;  // t_near in voxel coordinates
    float v_far = 1 / voxel_dim * t_far; // t_far in voxel coordinates
    Eigen::Vector3f ray_origin_coord_f;
    map.pointToVoxel(ray_origin_W, ray_origin_coord_f); // Origin in voxel coordinates

    // Current state of V in [voxel]
    Eigen::Vector3f V_max = Eigen::Vector3f::Ones();


    Eigen::Vector3f delta_V_map =
        octree.getSize() / ray_dir_W.array().abs(); // [voxel]/[-], potentionally dividing by 0

    Eigen::Vector3f map_frac = ray_origin_coord_f / octree.getSize();
    // V at which the map boundary gets crossed (separate V for each dimension x-y-z)
    Eigen::Vector3f v_map;
    if (ray_dir_W.x() < 0) {
        v_map.x() = map_frac.x() * delta_V_map.x();
    }
    else {
        v_map.x() = (1 - map_frac.x()) * delta_V_map.x();
    }

    if (ray_dir_W.y() < 0) {
        v_map.y() = map_frac.y() * delta_V_map.y();
    }
    else {
        v_map.y() = (1 - map_frac.y()) * delta_V_map.y();
    }

    if (ray_dir_W.z() < 0) {
        v_map.z() = map_frac.z() * delta_V_map.z();
    }
    else {
        v_map.z() = (1 - map_frac.z()) * delta_V_map.z();
    }

    // Maximum valid travelled distance in voxel is the minimum out of the far plane,
    // and the smallest distance that will make the ray cross the map boundary in either x, y or z ray_dir_W.
    v_far = std::min(std::min(std::min(v_map.x(), v_map.y()), v_map.z()) + v, v_far); // [voxel]
    t_far = voxel_dim * v_far;                                                        // [m]

    typename MapT::DataType data =
        se::visitor::getMaxData(octree, ray_origin_coord_f.cast<int>(), scale);

    while (get_field(data) > -0.2f && scale > 2) { // TODO Verify
        scale -= 1;
        data = se::visitor::getMaxData(octree, ray_origin_coord_f.cast<int>(), scale);
    }

    Eigen::Vector3f ray_coord_f = ray_origin_coord_f;

    while ((v + v_add) < v_far) {
        if (scale <= 2) {
            t = voxel_dim * (v + v_add - 4);
            return;
        }

        const int node_size = octantops::scale_to_size(scale);
        Eigen::Vector3i curr_node =
            node_size * (((ray_coord_f).array().floor()) / node_size).cast<int>();

        // Fraction of the current position in [voxel] in the current node along the x-, y- and z-axis
        Eigen::Vector3f node_frac = (ray_coord_f - curr_node.cast<float>()) / node_size;

        // Travelled distance needed in [voxel] to the whole node_size in x, y and z ray_dir_W
        Eigen::Vector3f delta_V = node_size / ray_dir_W.array().abs(); // [voxel]/[-]

        // Initalize V
        if (ray_dir_W.x() < 0) {
            V_max.x() = node_frac.x() * delta_V.x();
        }
        else {
            V_max.x() = (1 - node_frac.x()) * delta_V.x();
        }

        if (ray_dir_W.y() < 0) {
            V_max.y() = node_frac.y() * delta_V.y();
        }
        else {
            V_max.y() = (1 - node_frac.y()) * delta_V.y();
        }

        if (ray_dir_W.z() < 0) {
            V_max.z() = node_frac.z() * delta_V.z();
        }
        else {
            V_max.z() = (1 - node_frac.z()) * delta_V.z();
        }

        const float zero_depth_band = 1.0e-6f;
        for (int i = 0; i < 3; i++) {
            if (std::fabs(ray_dir_W[i]) < zero_depth_band) {
                V_max[i] = std::numeric_limits<float>::infinity();
            }
        }

        float V_min = std::min(std::min(V_max.x(), V_max.y()), V_max.z());

        v_add += V_min + 0.01;
        ray_coord_f = (v + v_add) * ray_dir_W + ray_origin_coord_f;

        data = se::visitor::getMaxData(octree, ray_coord_f.cast<int>(), scale);

        if (get_field(data) > -0.2f) {
            while (get_field(data) > -0.2f && scale > 2) {
                scale -= 1;
                data = se::visitor::getMaxData(octree, ray_coord_f.cast<int>(), scale);
            }
        }
        else {
            for (int s = scale + 1; s <= max_scale; s++) {
                data = se::visitor::getMaxData(octree, ray_coord_f.cast<int>(), s);

                if (get_field(data) > -0.2f) {
                    break;
                }
                scale += 1;
            }
        }
    }

    is_valid = false;
    return;
}



/**
 * \brief Compute the intersection point and scale for a given ray
 *
 * \param map           The reference to the map
 * \param ray_origin_W  The origin of the ray in world frame in [m]
 * \param ray_dir_W     The ray direction in world frame
 * \param p_near        The near plane distance in [m]
 * \param p_far         The far plane distance in [m]
 *
 * \return The Surface intersection point in [m] and scale
 */
template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const typename MapT::OctreeType& octree,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        float /* t_near */,
        float t_far)
{
    typedef typename MapT::DataType DataType;

    // Check if the ray origin is outside the map.
    // If so, compute the first point of contact with the map.
    // Stop if no intersection will occur (i.e. is_valid = false).
    bool is_valid = true;
    float t = compute_map_intersection(map, ray_origin_W, ray_dir_W, t_far, is_valid);

    if (!is_valid) {
        // Ray won't intersect with the map
        return std::nullopt;
    }

    const int max_scale = std::min(
        7,
        octree.getMaxScale()
            - 1); // Max possible free space skipped per iteration (node size = 2^max_scale)

    advance_ray(map, octree, ray_origin_W, ray_dir_W, t, t_far, max_scale, is_valid);

    if (!is_valid) {
        // Ray passes only through free space or intersects with the map before t_near or after t_far.
        return std::nullopt;
    }

    Eigen::Vector3f ray_pos_W = {};

    // first walk with largesteps until we found a hit
    const float step_size = map.getRes() / 2;
    std::optional<se::field_t> value_t;
    std::optional<se::field_t> value_tt;
    Eigen::Vector3f point_W_t = Eigen::Vector3f::Zero();
    Eigen::Vector3f point_W_tt = Eigen::Vector3f::Zero();
    int scale_tt = 0;

    value_t = find_valid_point(map, ray_origin_W, ray_dir_W, step_size, t_far, t, point_W_t);
    if (!value_t) {
        return std::nullopt;
    }
    t += step_size;

    // if we are not already in it
    if (*value_t <= MapT::DataType::surface_boundary) {
        for (; t < t_far; t += step_size) {
            ray_pos_W = ray_origin_W + ray_dir_W * t;
            DataType data = map.getData(ray_pos_W);
            if (!se::is_valid(data)) {
                t += step_size;
                value_t =
                    find_valid_point(map, ray_origin_W, ray_dir_W, step_size, t_far, t, point_W_t);
                if (!value_t) {
                    return Eigen::Vector4f::Zero();
                }

                if (*value_t > MapT::DataType::surface_boundary) {
                    break;
                }
                continue;
            }
            value_tt = std::optional<field_t>(get_field(data));
            point_W_tt = ray_pos_W;
            if (*value_tt > -0.2f) {
                value_tt = map.interpField(ray_pos_W, 0, &scale_tt);
                if (!value_tt) {
                    t += step_size;
                    value_t = find_valid_point(
                        map, ray_origin_W, ray_dir_W, step_size, t_far, t, point_W_t);
                    if (!value_t) {
                        return std::nullopt;
                    }
                    if (*value_t > MapT::DataType::surface_boundary) {
                        break;
                    }
                    continue;
                }
            }
            if (*value_tt > MapT::DataType::surface_boundary) { // got it, jump out of inner loop
                break;
            }
            value_t = value_tt;
            point_W_t = point_W_tt;
        }
        if (*value_tt > MapT::DataType::surface_boundary
            && *value_t < MapT::DataType::surface_boundary) {
            // We overshot. Need to move backwards for zero crossing.
            t = t
                - (point_W_tt - point_W_t).norm() * (*value_tt - MapT::DataType::surface_boundary)
                    / (*value_tt - *value_t);
            Eigen::Vector4f surface_point_W = (ray_origin_W + ray_dir_W * t).homogeneous();
            surface_point_W.w() = scale_tt;
            return surface_point_W;
        }
    }
    return std::nullopt;
}



template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const typename MapT::OctreeType& /* octree */,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        const float t_near,
        const float t_far)
{
    se::VoxelBlockRayIterator<MapT> ray(map, ray_origin_W, ray_dir_W, t_near, t_far);
    ray.next();

    float step = map.getRes();
    float largestep = MapT::BlockType::size * step;

    const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
    if (t_min <= 0.f) {
        return std::nullopt;
    }
    const float t_max = ray.tmax();

    float t = t_min;

    const float truncation_boundary =
        map.getRes() * map.getDataConfig().field.truncation_boundary_factor;

    if (t_near < t_max) {
        // first walk with largesteps until we found a hit
        float stepsize = largestep;
        Eigen::Vector3f point_W = ray_origin_W + ray_dir_W * t;
        typename MapT::DataType data = map.template getData<se::Safe::On>(point_W);
        float f_t = get_field(data);
        float f_tt = 0;
        int scale_tt = 0;
        // if we are not already in it
        if (f_t >= MapT::DataType::surface_boundary) {
            for (; t < t_far; t += stepsize) {
                data = map.template getData<se::Safe::On>(point_W);
                if (!se::is_valid(data)) {
                    stepsize = largestep;
                    point_W += stepsize * ray_dir_W;
                    continue;
                }

                f_tt = get_field(data);
                if (f_tt <= 0.1 && f_tt >= -0.5f) {
                    std::optional<se::field_t> field_value = [&]() -> std::optional<se::field_t> {
                        if constexpr (MapT::res_ == se::Res::Single) {
                            return map.interpField(point_W);
                        }
                        else {
                            return map.interpField(point_W, 0, &scale_tt);
                        }
                    }();
                    if (field_value) {
                        f_tt = *field_value;
                    }
                }

                if (f_tt < MapT::DataType::surface_boundary) {
                    break;
                } // got it, jump out of inner loop

                stepsize = std::max(f_tt * truncation_boundary, step);
                point_W += stepsize * ray_dir_W;
                f_t = f_tt;
            }
            // got it, calculate accurate intersection
            if (f_tt < MapT::DataType::surface_boundary) {
                t = t - stepsize * (f_tt - MapT::DataType::surface_boundary) / (f_tt - f_t);
                Eigen::Vector4f intersection_W = (ray_origin_W + ray_dir_W * t).homogeneous();
                intersection_W.w() = scale_tt;
                return intersection_W;
            }
        }
    }
    return std::nullopt;
}



template<typename MapT, typename SensorT>
void raycast_volume(const MapT& map,
                    const SensorT& sensor,
                    const Eigen::Isometry3f& T_WS,
                    se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                    se::Image<Eigen::Vector3f>& surface_normals_W,
                    se::Image<int8_t>& surface_scale,
                    se::Image<colour_t>* surface_colour,
                    se::Image<id_t>* surface_id)
{
    assert(surface_point_cloud_W.width() == surface_normals_W.width());
    assert(surface_point_cloud_W.height() == surface_normals_W.height());
    assert(surface_point_cloud_W.width() == surface_scale.width());
    assert(surface_point_cloud_W.height() == surface_scale.height());
    if constexpr (MapT::col_ == Colour::On) {
        if (surface_colour) {
            assert(surface_point_cloud_W.width() == surface_colour->width());
            assert(surface_point_cloud_W.height() == surface_colour->height());
        }
    }
    if constexpr (MapT::id_ == Id::On) {
        if (surface_id) {
            assert(surface_point_cloud_W.width() == surface_id->width());
            assert(surface_point_cloud_W.height() == surface_id->height());
        }
    }
    const typename MapT::OctreeType& octree = map.getOctree();
#pragma omp parallel for
    for (int y = 0; y < surface_point_cloud_W.height(); y++) {
#pragma omp simd
        for (int x = 0; x < surface_point_cloud_W.width(); x++) {
            const size_t idx = x + y * static_cast<size_t>(surface_point_cloud_W.width());
            Eigen::Vector3f ray_dir_S; //< Ray direction in sensor frame
            sensor.model.backProject(Eigen::Vector2f(x, y), &ray_dir_S);
            const Eigen::Vector3f ray_dir_W = T_WS.linear() * ray_dir_S.normalized();
            const std::optional<Eigen::Vector4f> surface_intersection_W =
                raycast(map,
                        octree,
                        T_WS.translation(),
                        ray_dir_W,
                        sensor.nearDist(ray_dir_S),
                        sensor.farDist(ray_dir_S));

            if (surface_intersection_W) {
                // Set surface scale
                surface_scale[idx] = static_cast<int>(surface_intersection_W->w());
                // Set surface point
                surface_point_cloud_W[idx] = surface_intersection_W->head<3>();
                // Compute surface normal from the field gradient.
                const std::optional<Eigen::Vector3f> surface_normal_W =
                    map.gradField(surface_intersection_W->head<3>());
                if (surface_normal_W) {
                    if constexpr (MapT::DataType::normals_along_gradient) {
                        surface_normals_W[idx] = surface_normal_W->normalized();
                    }
                    else {
                        surface_normals_W[idx] = -surface_normal_W->normalized();
                    }
                }
                else {
                    surface_normals_W[idx] = math::g_invalid_normal;
                }
                if constexpr (MapT::col_ == Colour::On) {
                    if (surface_colour) {
                        const auto result = map.interpColour(surface_intersection_W->head<3>());
                        (*surface_colour)[idx] = result ? *result : colour_t();
                    }
                }
                if constexpr (MapT::id_ == Id::On) {
                    if (surface_id) {
                        (*surface_id)[idx] = map.getData(surface_intersection_W->head<3>()).id.id;
                    }
                }
            }
            else {
                surface_point_cloud_W[idx] = Eigen::Vector3f::Zero();
                surface_normals_W[idx] = math::g_invalid_normal;
                if constexpr (MapT::col_ == Colour::On) {
                    if (surface_colour) {
                        (*surface_colour)[idx] = colour_t();
                    }
                }
                if constexpr (MapT::id_ == Id::On) {
                    if (surface_id) {
                        (*surface_id)[idx] = g_not_mapped;
                    }
                }
            }
        } // x
    } // y
}



template<typename GetDiffuseColourF>
void render_volume(se::Image<RGBA>& render,
                   const se::Image<Eigen::Vector3f>& surface_points_W,
                   const se::Image<Eigen::Vector3f>& surface_normals_W,
                   const GetDiffuseColourF get_diffuse_colour,
                   const Eigen::Vector3f& light_source_W,
                   const RGB ambient_light)
{
    assert(render.width() == surface_points_W.width());
    assert(render.height() == surface_points_W.height());
    assert(render.width() == surface_normals_W.width());
    assert(render.height() == surface_normals_W.height());
    const Eigen::Vector3f ambient_light_f(ambient_light.r, ambient_light.g, ambient_light.b);
#pragma omp parallel for
    for (size_t pixel_idx = 0; pixel_idx < render.size(); pixel_idx++) {
        RGBA colour;
        const Eigen::Vector3f& surface_normal_W = surface_normals_W[pixel_idx];
        if (surface_normal_W != math::g_invalid_normal && surface_normal_W.norm() > 0.f) {
            const Eigen::Vector3f& surface_point_W = surface_points_W[pixel_idx];
            const Eigen::Vector3f light_dir_W = (light_source_W - surface_point_W).normalized();
            assert(surface_normal_W.isApprox(surface_normal_W.normalized()));
            // The intensity must be 0 if the light is opposite the surface (negative dot product).
            const float intensity = std::max(surface_normal_W.dot(light_dir_W), 0.0f);
            const RGB rgb = get_diffuse_colour(pixel_idx);
            const Eigen::Vector3f diffuse = intensity * Eigen::Vector3f(rgb.r, rgb.g, rgb.b);
            Eigen::Vector3f col = diffuse + ambient_light_f;
            se::eigen::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Constant(255.0f));
            colour.r = col.x();
            colour.g = col.y();
            colour.b = col.z();
        }
        render[pixel_idx] = colour;
    }
}



template<typename MapT, typename SensorT>
Image<id_t> lookup_ids(const MapT& map,
                       const Image<float>& depth,
                       const SensorT& sensor,
                       const Eigen::Isometry3f& T_WC)
{
    Image<Eigen::Vector3f> point_cloud_C(depth.width(), depth.height());
    preprocessor::depth_to_point_cloud(point_cloud_C, depth, sensor);
    Image<id_t> ids(depth.width(), depth.height());
    for (size_t i = 0; i < point_cloud_C.size(); i++) {
        const bool valid_depth = depth[i] >= sensor.near_plane && depth[i] <= sensor.far_plane;
        const Eigen::Vector3f point_W = T_WC * point_cloud_C[i];
        if (valid_depth && map.contains(point_W)) {
            const auto& data = map.getData(point_W);
            ids[i] = is_valid(data) ? data.id.id : g_not_mapped;
        }
        else {
            ids[i] = g_not_mapped;
        }
    }
    return ids;
}

} // namespace raycaster
} // namespace se

#endif // SE_RAYCASTER_IMPL_HPP
