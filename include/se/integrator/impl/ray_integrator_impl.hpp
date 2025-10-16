/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAY_INTEGRATOR_IMPL_HPP
#define SE_RAY_INTEGRATOR_IMPL_HPP

#define PRINT_TIMING 0

namespace se {

template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
              SensorT>::RayIntegrator(MapType& map,
                                      const SensorT& sensor,
                                      const Eigen::Vector3f& ray,
                                      const Eigen::Isometry3f& T_WS,
                                      const timestamp_t timestamp,
                                      std::unordered_set<const OctantBase*>* const updated_octants) :
        map_(map),
        octree_(map.getOctree()),
        sensor_(sensor),
        node_set_(octree_.getBlockDepth()),
        config_(map),
        T_SW_(T_WS.inverse()),
        ray_(ray),
        last_visited_voxel_(Eigen::Vector3i::Constant(-1)),
        map_res_(map.getRes()),
        free_space_scale_(map_.getDataConfig().field.fs_integr_scale),
        timestamp_(timestamp),
        ray_dist_(ray_.norm()),
        tau_(compute_tau(ray_dist_, config_.tau_min, config_.tau_max, map_.getDataConfig())),
        three_sigma_(compute_three_sigma(ray_dist_,
                                         config_.sigma_min,
                                         config_.sigma_max,
                                         map_.getDataConfig()))
{
    updated_octants_ = updated_octants;
}

template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
bool RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                   SensorT>::resetIntegrator(const Eigen::Vector3f& ray,
                                             const Eigen::Isometry3f& T_WS,
                                             const timestamp_t timestamp,
                                             bool skip_ray)
{
    // Check if ray is expected to add new information. From the angle between sequential rays we compute the
    // distance between two measurements and check if that is smaller than the voxel / block at the
    // current integration scale
    if (skip_ray) {
        if ((ray - ray_).norm() < sqrt(3.0f) * map_res_) {
            // Make sure that measurements can only be jumped in case of not too large deltas
            const float angle_to_prev_ray = std::acos(ray.normalized().dot(ray_.normalized()));
            const float dist_to_prev_update = std::sin(angle_to_prev_ray) * ray.norm();
            if (dist_to_prev_update
                < map_res_ * octantops::scale_to_size(computed_integration_scale_)) {
                return false;
            }
        }
    }

    ray_ = ray;
    ray_dist_ = ray.norm();
    T_SW_ = T_WS.inverse();
    computed_integration_scale_ = 0;
    tau_ = compute_tau(ray_dist_, config_.tau_min, config_.tau_max, map_.getDataConfig());
    three_sigma_ =
        compute_three_sigma(ray_dist_, config_.sigma_min, config_.sigma_max, map_.getDataConfig());
    last_visited_voxel_ = Eigen::Vector3i::Constant(-1);
    timestamp_ = timestamp;
    return true;
}


template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                   SensorT>::operator()()
{
    /// (0) Get Root Pointer
    se::OctantBase* root_ptr = octree_.getRoot();

    /// (1) Allocate all 8 children first level (if not yet allocated)
    octree_.allocateChildren(static_cast<NodeType*>(root_ptr));
    
    /// (2) Check validity of measured value (sensor near plane)
    if (ray_dist_ < sensor_.near_plane) {
        return;
    }

    /// (3) Determine maximum update distance along the ray, cut to maximum of sensor range (far plane)
    const Eigen::Vector3f ray_dir_S = ray_.normalized();
    float max_update_dist = std::min(ray_dist_ + tau_, sensor_.far_plane);
    float min_update_dist = sensor_.near_plane;

    /// (4) Check if ray crosses map boundaries (Origin or measurement or both) are outside of the map
    const Eigen::Isometry3f T_WS = T_SW_.inverse();
    const Eigen::Vector3f ray_origin_in_W = T_WS.translation();
    const Eigen::Vector3f ray_dir_in_W = T_WS.rotation() * ray_.normalized();
    const Eigen::Vector3f ray_end_in_W = T_WS * ray_;

    const bool origin_in_map = map_.contains(ray_origin_in_W);
    const bool measurement_in_map = map_.contains(ray_end_in_W);

    if(!origin_in_map || (origin_in_map && !measurement_in_map)){
        // The ray is crossing a map boundary, determine crossing points
        se::VoxelBlockRayIterator<MapType> rayIterator(map_, ray_origin_in_W, ray_dir_in_W, sensor_.near_plane, sensor_.far_plane);
        const float tmax = rayIterator.tmax();
        const float tmin = rayIterator.tmin();
        if (tmax < tmin) {
            // No valid intersections of the map with map boundaries.
            return;
        }

        // tmax: Exit point of ray
        if (tmax == sensor_.far_plane) {
            // Map boundary where ray exits is further than far plane
            max_update_dist = sensor_.far_plane; // this should be already implicitly handled
        }
        else if (tmax < sensor_.far_plane) {
            // Map boundary between near and far plane, restrict update distance to tmax
            max_update_dist = tmax;
        }

        // Handle Cases with origin outside of map -> tmin: entrance point
        if (!origin_in_map) {
            if (tmin > sensor_.near_plane) {
                // Ray enteres cube at distance tmin
                min_update_dist = tmin;
            }
            else if (tmin == sensor_.near_plane) {
                // Origin closer to map than near plane: "entry point" will be inside map
                min_update_dist = sensor_.near_plane; // this should be already implicitly handled
            }
        }
    }

    /// (5) "Inverted" Ray-Casting. Starting from extended ray measurement towards the sensor
    Eigen::Vector3f r_i_S = max_update_dist * ray_dir_S;
    Eigen::Vector3f r_i_W;

    while (r_i_S.norm() > min_update_dist) {
        // Compute if in free space
        se::RayState ray_state = computeVariance(r_i_S.norm());

        // Convert from sensor to Map World Frame
        r_i_W = T_WS * r_i_S;
        Eigen::Vector3i voxel_coord;
        if (!map_.template pointToVoxel<se::Safe::On>(r_i_W, voxel_coord)) {
            // Outside Map, before we had a break, now we have to check when we are getting into the map
            r_i_S -=
                0.5 * map_res_ * octantops::scale_to_size(free_space_scale_) * ray_dir_S;
            continue;
            // break;
        }

        if (voxel_coord == last_visited_voxel_) {
            // can jump to next sample
            r_i_S -=
                0.5 * map_res_ * octantops::scale_to_size(computed_integration_scale_) * ray_dir_S;
            continue;
        }
        last_visited_voxel_ = voxel_coord;

        if ((*this)(r_i_S, voxel_coord, ray_state, root_ptr)) {
            r_i_S -= 0.5 * map_res_ * octantops::scale_to_size(computed_integration_scale_) * ray_dir_S;
            // Nothing else to do...
        }
        else {
            break; // We hit a free space Block / voxel that was already updated, continue
        }
    }
}

template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                   SensorT>::propagateBlocksToCoarsestScale()
{
#pragma omp parallel for num_threads(3)
    for (size_t i = 0; i < updated_blocks_vector_.size(); i++) {
        se::ray_integrator::propagate_block_to_coarsest_scale<BlockType>(updated_blocks_vector_[i]);
    }
}

template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
se::RayState RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                           SensorT>::computeVariance(const float ray_step_depth)
{
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff = (ray_step_depth - ray_dist_);

    if (z_diff < -three_sigma_) { // Guaranteed free Space
        return se::RayState::FreeSpace;
    }
    else if (z_diff < tau_ / 2.0) {
        return se::RayState::Transition;
    }
    else if (
        z_diff
        < tau_) { // within surface thickness => Occupied ToDo: remove distinction transition occupied
        return se::RayState::Occupied;
    }
    else { // Behind Surface
        return se::RayState::Undefined;
    }
}


template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, bool>
RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
              SensorT>::operator()(const Eigen::Vector3f& ray_sample,
                                   const Eigen::Vector3i& voxel_coord,
                                   se::RayState rayState,
                                   se::OctantBase* octant_ptr)
{
    /// (1.a) Determine closest currently allocated octant
    se::OctantBase* finest_octant_ptr = se::fetcher::finest_octant<OctreeType>(
        voxel_coord, 0, octant_ptr); // up to scale 0 meaning down to finest resolution

    /// (1.b) Check if block level is already reached
    if (finest_octant_ptr->is_block) {
        /// ALREADY BLOCK-LEVEL -> DO update here
        /// (2.a) Determine Integration Scale
        // compute integration scale
        BlockType* block_ptr = static_cast<BlockType*>(finest_octant_ptr);
        // The last integration scale
        const int last_scale = (block_ptr->min_scale == -1) ? 0 : block_ptr->current_scale;

        // Compute the point of the block centre in the sensor frame ToDo: is this needed? Can't we just use the point in sensor frame?
        const unsigned int block_size = BlockType::size;
        const Eigen::Vector3i block_coord = block_ptr->coord;
        Eigen::Vector3f block_centre_point_W;
        map_.voxelToPoint(block_coord, block_size, block_centre_point_W);
        const Eigen::Vector3f block_centre_point_C = T_SW_ * block_centre_point_W;

        // Determine Integration Scale
        int computed_integration_scale;
        if (rayState == se::RayState::FreeSpace && block_ptr->min_scale != -1) {
            // The block has been already updated, do the free space update at its
            // current scale.
            computed_integration_scale = block_ptr->current_scale;
        } else {
            computed_integration_scale = sensor_.blockIntegrationScale(
                block_centre_point_C, map_res_, last_scale, block_ptr->min_scale, block_ptr->max_scale);
            if (rayState == se::RayState::FreeSpace) {
                // The block hasn't been updated before, update as free at a
                // scale no finer than free_space_scale_.
                computed_integration_scale = std::max(free_space_scale_, computed_integration_scale);
            }
        }

        /// (2.b) Update Block
        updateBlock(finest_octant_ptr,
                    const_cast<Eigen::Vector3i&>(voxel_coord),
                    computed_integration_scale,
                    ray_sample.norm());
        computed_integration_scale_ = computed_integration_scale;

        /// (2.c) Save block for later up-propagation (only save once)
        if (updated_blocks_set_.count(finest_octant_ptr) == 0) {
            updated_blocks_set_.insert(finest_octant_ptr);
            updated_blocks_vector_.push_back(finest_octant_ptr);
        }
        if (updated_octants_) {
            updated_octants_->insert(finest_octant_ptr);
        }
    }
    else {
        auto* node_ptr = static_cast<NodeType*>(finest_octant_ptr);
        if (rayState == se::RayState::FreeSpace && node_ptr->max_data.field.observed) {
            return false;
        }
        octree_.allocateChildren(node_ptr);
        return (*this)(ray_sample, voxel_coord, rayState, finest_octant_ptr);
    }

    return true;
}


template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                   SensorT>::updateBlock(se::OctantBase* octant_ptr,
                                         Eigen::Vector3i& voxel_coords,
                                         int desired_scale,
                                         float sample_dist)
{
    if (sample_dist < 1e-08)
        return;
    // Block (pointer) to be updated
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    const Eigen::Vector3i block_coord = block_ptr->coord; /// < Coordinates of block to be updated

    // Set timestamp of the current block
    timestamp_t previous_time_stamp = block_ptr->timestamp;
    const bool is_already_integrated = (previous_time_stamp == timestamp_);
    if (!is_already_integrated) {
        block_ptr->timestamp = timestamp_;
    }


    // The last integration scale
    // -- Nothing integrated yet (-1) => set to desired_scale
    // -- otherwise current scale of block ptr
    const int last_scale = (block_ptr->min_scale == -1) ? desired_scale : block_ptr->current_scale;
    int integration_scale = last_scale;


    // Case 1: Nothing integrated yet:
    if (block_ptr->min_scale == -1) { // nothing inegrated yet
        // Make sure the block is allocated up to the integration scale
        block_ptr->allocateDownTo(integration_scale);
        block_ptr->init_data = DataType();
    }
    else if (desired_scale < last_scale) {
        se::ray_integrator::propagate_block_down_to_scale<BlockType>(block_ptr, desired_scale);

        // set new scale and min scale
        integration_scale = desired_scale;
    }
    else if (!is_already_integrated && (desired_scale > last_scale)) {
        se::ray_integrator::propagate_block_to_scale<BlockType>(block_ptr, desired_scale);
        integration_scale = desired_scale;
        block_ptr->deleteUpTo(integration_scale);
    }

    /// Determine voxels to be updated
    const unsigned int integration_stride = 1
        << integration_scale; // 1 for scale 0, 2 for scale 1, 4 for scale 2 etc.
    ///< Number of voxels per dimension that has to be updated at integration scale
    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    ///< Size of the block at integration scale
    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);
    ///< Squared size of the block at integration scale

    const Eigen::Vector3i offset_coords = (voxel_coords - block_coord) / integration_stride;

    const int voxel_idx = offset_coords.x() + size_at_integration_scale_li * offset_coords.y()
        + size_at_integration_scale_sq * offset_coords.z();
    DataType* data_at_scale = block_ptr->blockDataAtScale(integration_scale);
    auto& voxel_data = data_at_scale[voxel_idx];
    float range_diff = sample_dist - ray_dist_;
    ray_integrator::update_voxel(voxel_data, range_diff, tau_, three_sigma_, map_.getDataConfig());
}


template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                   SensorT>::propagateToRoot()
{
    // Retrieving Parent Nodes for all updated blocks
    for (const auto& octant_ptr : updated_blocks_vector_) {
        BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
        if (block_ptr->parent()) {
            node_set_[octree_.getBlockDepth() - 1].insert(block_ptr->parent());
        }
    }

    for (int d = octree_.getBlockDepth() - 1; d > 0; d--) // TODO: block depth - 1?
    {
        std::set<se::OctantBase*>::iterator it;
        for (it = node_set_[d].begin(); it != node_set_[d].end(); ++it) {
            se::OctantBase* octant_ptr = *it;
            if (octant_ptr->timestamp == timestamp_) {
                continue;
            }

            if (octant_ptr->parent()) {
                auto node_data = ray_integrator::propagate_to_parent_node<NodeType, BlockType>(
                    octant_ptr, timestamp_);
                node_set_[d - 1].insert(octant_ptr->parent());
                if (updated_octants_) {
                    updated_blocks_set_.insert(octant_ptr);
                    updated_octants_->insert(octant_ptr);
                }

                // If all nodes free space, delete children and just leave coarser resolution
                if (node_data.field.observed
                    && get_field(node_data) <= 0.95 * MapType::DataType::FieldType::min_occupancy) {
                    auto* node_ptr = static_cast<NodeType*>(octant_ptr);
                    if (updated_octants_) {
                        for (int i = 0; i < 8; i++) {
                            OctantBase* const child_ptr = node_ptr->getChild(i);
                            if (child_ptr) {
                                updated_blocks_set_.erase(child_ptr);
                                updated_octants_->erase(octant_ptr);
                            }
                        }
                    }
                    octree_.deleteChildren(node_ptr);
                }

            } // if parent
        } // nodes at depth d
    } // depth d

    ray_integrator::propagate_to_parent_node<NodeType, BlockType>(octree_.getRoot(), timestamp_);
}

} // namespace se

#endif //SE_RAY_INTEGRATOR_IMPL_HPP
