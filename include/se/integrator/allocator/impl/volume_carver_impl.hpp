/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VOXEL_CARVER_IMPL_HPP
#define SE_VOXEL_CARVER_IMPL_HPP



namespace se {



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
             SensorT>::VolumeCarver(MapType& map,
                                    const SensorT& sensor,
                                    const se::Image<float>& depth_img,
                                    const se::Image<float>& depth_sigma_img,
                                    const Eigen::Isometry3f& T_WS,
                                    const timestamp_t /* timestamp */) :
        map_(map),
        octree_(map.getOctree()),
        sensor_(sensor),
        depth_pooling_img_(depth_img),
        sigma_pooling_img_(depth_sigma_img),
        T_SW_(T_WS.inverse()),
        map_res_(map.getRes()),
        config_(map),
        max_depth_value_(
            std::min(sensor.far_plane, depth_pooling_img_.maxValue() + config_.tau_max)),
        zero_depth_band_(1.0e-6f),
        size_to_radius_(std::sqrt(3.0f) / 2.0f)
{
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
VolumeCarverAllocation
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
             SensorT>::operator()()
{
    NodeType* root_ptr = static_cast<NodeType*>(octree_.getRoot());
    const int child_size = root_ptr->size / 2;
    octree_.allocateChildren(root_ptr);
    // Launch on the root node's children.
#pragma omp parallel for
    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        (*this)(root_ptr->getChildCoord(child_idx), child_size, 1, root_ptr->getChild(child_idx));
    }

    // Extend the octree AABB to contain all leaf nodes. See se::Octree::aabbExtend() on why this
    // can't be done on the octree side.
    for (const OctantBase* octant : allocation_list_.node_list) {
        if (octant->isLeaf()) {
            octree_.aabbExtend(octant->coord, static_cast<const NodeType*>(octant)->size);
        }
    }

    return allocation_list_;
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>, SensorT>::
    crossesFrustum(std::vector<srl::projection::ProjectionStatus>& proj_corner_stati)
{
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        if (proj_corner_stati[corner_idx] == srl::projection::ProjectionStatus::Successful) {
            return true;
        }
    }
    return false;
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                  SensorT>::cameraInNode(const Eigen::Vector3i& node_coord,
                                         const int node_size,
                                         const Eigen::Isometry3f& T_WS)
{
    assert(node_size > 0);
    Eigen::Vector3f voxel_coord_f;
    map_.pointToVoxel(T_WS.translation(), voxel_coord_f);
    const Eigen::Array3f lower = node_coord.cast<float>().array();
    const Eigen::Array3f upper = lower + node_size;
    return (voxel_coord_f.array() >= lower).all() && (voxel_coord_f.array() < upper).all();
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
se::VarianceState
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
             SensorT>::computeVariance(const float depth_value_min,
                                       const float depth_value_max,
                                       const float node_dist_min_m,
                                       const float node_dist_max_m,
                                       const float std_dev_max)
{
    assert(depth_value_min <= depth_value_max);
    assert(node_dist_min_m <= node_dist_max_m);
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff_max = (node_dist_max_m - depth_value_min); // * proj_scale;
    const float z_diff_min = (node_dist_min_m - depth_value_max); // * proj_scale;

    const float tau_max =
        compute_tau(depth_value_max, config_.tau_min, config_.tau_max, map_.getDataConfig());
    const float three_sigma_min = 3.0f * std_dev_max;

    if (z_diff_min > 1.25 * tau_max) { // behind of surface
        return se::VarianceState::Undefined;
    }
    else if (z_diff_max < -1.25 * three_sigma_min) { // guranteed free space
        return se::VarianceState::Constant;
    }
    else {
        return se::VarianceState::Gradient;
    }
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, PinholeCamera>::value, void>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
             SensorT>::operator()(const Eigen::Vector3i& octant_coord,
                                  const int octant_size,
                                  const int octant_depth,
                                  se::OctantBase* octant_ptr)
{
    assert(octant_size > 0);
    assert(octant_depth >= 0);
    assert(octant_ptr);
    /// Approximate max and min depth to quickly check if the node is behind the camera or maximum depth.
    // Compute the node centre's depth in the camera frame

    /// CHANGE: USE voxelToPoint(...)
    Eigen::Vector3f node_centre_point_W;
    map_.voxelToPoint(octant_coord, octant_size, node_centre_point_W);

    const Eigen::Vector3f node_centre_point_S = T_SW_ * node_centre_point_W;

    // Extend and reduce the depth by the sphere radius covering the entire cube
    const float approx_depth_value_max =
        node_centre_point_S.z() + octant_size * size_to_radius_ * map_res_;
    const float approx_depth_value_min =
        node_centre_point_S.z() - octant_size * size_to_radius_ * map_res_;

    /// CASE 0.1 (OUT OF BOUNDS): Block is behind the camera or behind the maximum depth value
    if (approx_depth_value_min > max_depth_value_
        || approx_depth_value_max < zero_depth_band_) { // TODO: Alternative sensor_.near_plane.
        return;
    }

    // Compute the 8 corners of the node to be evaluated
    /// CHANGE: USE voxelToCornerPoints(...)
    Eigen::Matrix<float, 3, 8> node_corner_points_W;
    map_.voxelToCornerPoints(octant_coord, octant_size, node_corner_points_W);
    Eigen::Matrix<float, 3, 8> node_corner_points_S = T_SW_ * node_corner_points_W;

    Eigen::VectorXi node_corners_infront(8);
    node_corners_infront << 1, 1, 1, 1, 1, 1, 1, 1;
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        if (node_corner_points_S(2, corner_idx) < zero_depth_band_) {
            node_corners_infront(corner_idx) = 0;
        }
    }

    int num_node_corners_infront = node_corners_infront.sum();

    /// CASE 0.2 (OUT OF BOUNDS): Node is behind the camera.
    if (num_node_corners_infront == 0) {
        return;
    }

    // Project the 8 corners into the image plane
    Eigen::Matrix2Xf proj_node_corner_pixels_f(2, 8);
    const Eigen::VectorXf node_corners_diff = node_corner_points_S.row(2);
    std::vector<srl::projection::ProjectionStatus> proj_node_corner_stati;
    sensor_.model.projectBatch(
        node_corner_points_S, &proj_node_corner_pixels_f, &proj_node_corner_stati);

    /// Approximate a 2D bounding box covering the projected node in the image plane.
    bool should_split = false;
    bool projects_inside = false;
    se::VarianceState variance_state = se::VarianceState::
        Gradient; ///<< -1 := low variance infront of the surface, 0 := high variance, 1 = low_variance behind the surface.
    se::Pixel pooling_pixel = se::Pixel::
        crossingUnknownPixel(); ///<< min, max pixel batch depth + crossing frustum state + contains unknown values state.

    if (octant_depth < octree_.getBlockDepth() + 1) {
        if (num_node_corners_infront < 8) {
            /// CASE 1 (CAMERA IN NODE):
            if (cameraInNode(octant_coord, octant_size, T_SW_.inverse())) {
                should_split = true;
                /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary
            }
            else if (crossesFrustum(proj_node_corner_stati)) {
                should_split = true;
                /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary without a corner reprojecting
            }
            else if (sensor_.sphereInFrustumInf(node_centre_point_S,
                                                octant_size * size_to_radius_ * map_res_)) {
                should_split = true;
            }
            else {
                return;
            }
        }
        else {
            // Clamp to avoid overflow/underflow when casting to int.
            proj_node_corner_pixels_f = (proj_node_corner_pixels_f.array() > 1e7f)
                                            .select(1e7f, proj_node_corner_pixels_f.array());
            proj_node_corner_pixels_f = (proj_node_corner_pixels_f.array() < -1e7f)
                                            .select(-1e7f, proj_node_corner_pixels_f.array());

            // Compute the minimum and maximum pixel values to generate the bounding box
            const Eigen::Vector2i img_bb_min =
                proj_node_corner_pixels_f.rowwise().minCoeff().cast<int>();
            const Eigen::Vector2i img_bb_max =
                proj_node_corner_pixels_f.rowwise().maxCoeff().cast<int>();
            const float node_dist_min_m = node_corners_diff.minCoeff();
            const float node_dist_max_m = node_corners_diff.maxCoeff();

            pooling_pixel = depth_pooling_img_.conservativeQuery(img_bb_min, img_bb_max);

            /// CASE 0.3 (OUT OF BOUNDS): The node is outside frustum (i.e left, right, below, above) or
            ///                           all pixel values are unknown -> return intermediately
            if (pooling_pixel.status_known == se::Pixel::statusKnown::unknown) {
                return;
            }

            /// CASE 0.4 (OUT OF BOUNDS): The node is behind surface
            if (node_dist_min_m
                > pooling_pixel.max + config_.tau_max) { // TODO: Can be changed to node_dist_max_m?
                return;
            }

            const se::Pixel sigma_pix =
                sigma_pooling_img_.conservativeQuery(img_bb_min, img_bb_max);
            variance_state = computeVariance(pooling_pixel.min,
                                             pooling_pixel.max,
                                             node_dist_min_m,
                                             node_dist_max_m,
                                             sigma_pix.max);

            /// CASE 1 (REDUNDANT DATA): Depth values in the bounding box are far away from the node or unknown (1).
            ///                          The node to be evaluated is free (2) and fully observed (3),
            if (variance_state != se::VarianceState::Gradient) {
                typename OctreeType::DataType child_data = (octant_ptr->is_block)
                    ? static_cast<BlockType*>(octant_ptr)->maxData()
                    : static_cast<NodeType*>(octant_ptr)->data();

                // Check if the child is fully observed (i.e. all children are observed) // TODO: incooperate MAX occupancy
                if (child_data.field.observed
                    && get_field(child_data)
                        <= 0.95 * MapType::DataType::FieldType::min_occupancy) {
                    return;
                }
            }

            /// CASE 2 (FRUSTUM BOUNDARY): The node is crossing the frustum boundary
            if (pooling_pixel.status_crossing == se::Pixel::statusCrossing::crossing) {
                should_split = true;
            }

            /// CASE 3: The node is inside the frustum, but projects into partially known pixel
            else if (pooling_pixel.status_known == se::Pixel::statusKnown::part_known) {
                should_split = true;
            }

            /// CASE 4: The node is inside the frustum with only known data + node has a potential high variance
            else if (variance_state == se::VarianceState::Gradient) {
                should_split = true;
            }

            projects_inside = (pooling_pixel.status_known == se::Pixel::known);
        }
    }

    if (should_split) {
        // Returns a pointer to the according node if it has previously been allocated.
        if (octant_ptr->is_block) { // Evaluate the node directly if it is a voxel block
#pragma omp critical(block_lock)
            { // Add voxel block to voxel block list for later update and up-propagation
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else {
            NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);
            const int child_size = node_ptr->size / 2;
            // Split! Start recursive process
            octree_.allocateChildren(node_ptr);
#pragma omp parallel for
            for (int child_idx = 0; child_idx < 8; ++child_idx) {
                (*this)(node_ptr->getChildCoord(child_idx),
                        child_size,
                        octant_depth + 1,
                        node_ptr->getChild(child_idx));
            }
        }
    }
    else {
        assert(octant_depth);

        if (octant_ptr->is_block) {
#pragma omp critical(block_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else if (variance_state == se::VarianceState::Constant) {
#pragma omp critical(node_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.node_list.push_back(octant_ptr);
            }
        } // else node has low variance behind surface (ignore)
    }
}



template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, se::OusterLidar>::value, void>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
             SensorT>::operator()(const Eigen::Vector3i& octant_coord,
                                  const int octant_size,
                                  const int octant_depth,
                                  se::OctantBase* octant_ptr)
{
    assert(octant_size > 0);
    assert(octant_depth >= 0);
    assert(octant_ptr);
    /// Approximate max and min depth to quickly check if the node is behind the camera or maximum depth.
    // Compute the node centre's depth in the camera frame

    /// CHANGE: USE voxelToPoint(...)
    Eigen::Vector3f node_centre_point_W;
    map_.voxelToPoint(octant_coord, octant_size, node_centre_point_W);

    const Eigen::Vector3f node_centre_point_S = T_SW_ * node_centre_point_W;

    // Extend and reduce the depth by the sphere radius covering the entire cube
    const float approx_depth_value_max =
        node_centre_point_S.norm() + octant_size * size_to_radius_ * map_res_;
    const float approx_depth_value_min =
        node_centre_point_S.norm() - octant_size * size_to_radius_ * map_res_;

    /// CASE 0.1 (OUT OF BOUNDS): Block is behind the camera or behind the maximum depth value
    if (approx_depth_value_min > max_depth_value_) {
        return;
    }

    /// Approximate a 2D bounding box covering the projected node in the image plane.
    bool should_split = false;
    bool projects_inside = false;
    se::VarianceState variance_state = se::VarianceState::
        Gradient; ///<< -1 := low variance infront of the surface, 0 := high variance, 1 = low_variance behind the surface.
    se::Pixel pooling_pixel = se::Pixel::
        crossingUnknownPixel(); ///<< min, max pixel batch depth + crossing frustum state + contains unknown values state.

    if (octant_depth < octree_.getBlockDepth() + 1) {
        /// CASE 1 (CAMERA IN NODE):
        if (cameraInNode(octant_coord, octant_size, T_SW_.inverse())) {
            should_split = true;
            /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary
        }
        else {
            // Compute the 8 corners of the node to be evaluated
            Eigen::Matrix<float, 3, 8> node_corner_points_W;
            map_.voxelToCornerPoints(octant_coord, octant_size, node_corner_points_W);
            Eigen::Matrix<float, 3, 8> node_corner_points_S = T_SW_ * node_corner_points_W;

            // Project the 8 corners into the image plane
            Eigen::Matrix2Xf proj_node_corner_pixels_f(2, 8);
            std::vector<srl::projection::ProjectionStatus> proj_node_corner_stati;
            sensor_.model.projectBatch(
                node_corner_points_S, &proj_node_corner_pixels_f, &proj_node_corner_stati);

            Eigen::Vector2f proj_node_centre_pixel_f;
            sensor_.model.project(node_centre_point_S, &proj_node_centre_pixel_f);

            float proj_node_centre_pixel_u_f = proj_node_centre_pixel_f.x();
            float proj_node_centre_pixel_u_comp_f =
                proj_node_centre_pixel_u_f + sensor_.model.imageWidth() / 2;

            if (proj_node_centre_pixel_u_comp_f > sensor_.model.imageWidth() - 0.5) {
                proj_node_centre_pixel_u_comp_f =
                    proj_node_centre_pixel_u_comp_f - sensor_.model.imageWidth();
            }

            Eigen::VectorXf proj_node_corner_pixels_u_f(8);
            for (int i = 0; i < 8; i++) {
                if (proj_node_centre_pixel_u_f < proj_node_centre_pixel_u_comp_f
                    && proj_node_corner_pixels_f(0, i) > proj_node_centre_pixel_u_f
                    && proj_node_corner_pixels_f(0, i) > proj_node_centre_pixel_u_comp_f) {
                    proj_node_corner_pixels_u_f(i) =
                        proj_node_corner_pixels_f(0, i) - sensor_.model.imageWidth();
                }
                else if (proj_node_centre_pixel_u_f > proj_node_centre_pixel_u_comp_f
                         && proj_node_corner_pixels_f(0, i) < proj_node_centre_pixel_u_f
                         && proj_node_corner_pixels_f(0, i) < proj_node_centre_pixel_u_comp_f) {
                    proj_node_corner_pixels_u_f(i) =
                        proj_node_corner_pixels_f(0, i) + sensor_.model.imageWidth();
                }
                else {
                    proj_node_corner_pixels_u_f(i) = proj_node_corner_pixels_f(0, i);
                }
            }
            Eigen::VectorXf proj_node_corner_pixels_v_f = proj_node_corner_pixels_f.row(1);

            int u_min = proj_node_corner_pixels_u_f.minCoeff();
            int u_max = proj_node_corner_pixels_u_f.maxCoeff();
            int v_min = proj_node_corner_pixels_v_f.minCoeff();
            int v_max = proj_node_corner_pixels_v_f.maxCoeff();

            // Compute the minimum and maximum pixel values to generate the bounding box
            const Eigen::Vector2i img_bb_min(u_min, v_min);
            const Eigen::Vector2i img_bb_max(u_max, v_max);

            pooling_pixel = depth_pooling_img_.conservativeQuery(img_bb_min, img_bb_max);

            /// CASE 0.3 (OUT OF BOUNDS): The node is outside frustum (i.e left, right, below, above) or
            ///                           all pixel values are unknown -> return intermediately
            if (pooling_pixel.status_known == se::Pixel::statusKnown::unknown) {
                return;
            }

            /// CASE 0.4 (OUT OF BOUNDS): The node is behind surface
            if (approx_depth_value_min
                > pooling_pixel.max + config_.tau_max) { // TODO: Can be changed to node_dist_max_m?
                return;
            }

            const se::Pixel sigma_pix =
                sigma_pooling_img_.conservativeQuery(img_bb_min, img_bb_max);
            variance_state = computeVariance(pooling_pixel.min,
                                             pooling_pixel.max,
                                             approx_depth_value_min,
                                             approx_depth_value_max,
                                             sigma_pix.max);

            /// CASE 1 (REDUNDANT DATA): Depth values in the bounding box are far away from the node or unknown (1).
            ///                          The node to be evaluated is free (2) and fully observed (3)
            if (variance_state != se::VarianceState::Gradient) {
                typename OctreeType::DataType child_data = (octant_ptr->is_block)
                    ? static_cast<BlockType*>(octant_ptr)->maxData()
                    : static_cast<NodeType*>(octant_ptr)->data();

                // Check if the child is fully observed (i.e. all children are observed) // TODO: incooperate MAX occupancy
                if (child_data.field.observed
                    && get_field(child_data)
                        <= 0.95 * MapType::DataType::FieldType::min_occupancy) {
                    return;
                }
            }

            /// CASE 2 (FRUSTUM BOUNDARY): The node is crossing the frustum boundary
            if (pooling_pixel.status_crossing == se::Pixel::statusCrossing::crossing) {
                should_split = true;
            }

            /// CASE 3: The node is inside the frustum, but projects into partially known pixel
            else if (pooling_pixel.status_known == se::Pixel::statusKnown::part_known) {
                should_split = true;
            }

            /// CASE 4: The node is inside the frustum with only known data + node has a potential high variance
            else if (variance_state == se::VarianceState::Gradient) {
                should_split = true;
            }

            else {
            }

            projects_inside = (pooling_pixel.status_known == se::Pixel::known);
        }
    }
    else {
    }

    if (should_split) {
        // Returns a pointer to the according node if it has previously been allocated.
        if (octant_ptr->is_block) { // Evaluate the node directly if it is a voxel block
#pragma omp critical(block_lock)
            { // Add voxel block to voxel block list for later update and up-propagation
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else {
            NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);
            const int child_size = node_ptr->size / 2;
            // Split! Start recursive process
            octree_.allocateChildren(node_ptr);
#pragma omp parallel for
            for (int child_idx = 0; child_idx < 8; ++child_idx) {
                (*this)(node_ptr->getChildCoord(child_idx),
                        child_size,
                        octant_depth + 1,
                        node_ptr->getChild(child_idx));
            }
        }
    }
    else {
        assert(octant_depth);

        if (octant_ptr->is_block) {
#pragma omp critical(block_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else if (variance_state == se::VarianceState::Constant) {
#pragma omp critical(node_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.node_list.push_back(octant_ptr);
            }
        } // else node has low variance behind surface (ignore)
    }
}



} // namespace se

#endif // SE_VOXEL_CARVER_IMPL_HPP
