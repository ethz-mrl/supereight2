/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAY_INTEGRATOR_CORE_IMPL_HPP
#define SE_RAY_INTEGRATOR_CORE_IMPL_HPP


namespace se {

namespace ray_integrator {

template<typename DataT, typename ConfigT>
bool update_voxel(DataT& data,
                  const float range_diff,
                  const float tau,
                  const float three_sigma,
                  const ConfigT config)
{
    float sample_value;

    if (range_diff < -three_sigma) {
        sample_value = config.field.log_odd_min;
    }
    else if (range_diff < tau / 2) {
        sample_value =
            std::min(config.field.log_odd_min
                         - config.field.log_odd_min / three_sigma * (range_diff + three_sigma),
                     config.field.log_odd_max);
    }
    else if (range_diff < tau) {
        sample_value =
            std::min(-config.field.log_odd_min * tau / (2 * three_sigma), config.field.log_odd_max);
    }
    else {
        return false;
    }

    const bool newly_observed = !data.field.observed;
    data.field.update(sample_value, config.field.max_weight);
    return newly_observed;
}


template<typename DataT, typename ConfigT>
bool free_voxel(DataT& voxel_data, const ConfigT config)
{
    const bool newly_observed = !voxel_data.field.observed;
    voxel_data.field.update(config.field.log_odd_min, config.field.max_weight);
    return newly_observed;
}



template<typename NodeT, typename BlockT>
typename NodeT::DataType propagate_to_parent_node(OctantBase* octant_ptr,
                                                  const timestamp_t timestamp)
{
    assert(octant_ptr);
    assert(!octant_ptr->is_block);

    NodeT& node = *static_cast<NodeT*>(octant_ptr);
    node.timestamp = timestamp;

    field_t max_occupancy = std::numeric_limits<field_t>::lowest();
    field_t max_mean_occupancy = 0;
    weight_t max_weight = 0;
    int max_data_count = 0;

    field_t min_mean_occupancy = 0;
    weight_t min_weight = 0;
    field_t min_occupancy = std::numeric_limits<field_t>::max();
    int min_data_count = 0;

    int observed_count = 0;

    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        const OctantBase* child_ptr = node.getChild(child_idx);

        if (!child_ptr) {
            continue;
        }

        const auto& child_min_data = (child_ptr->is_block)
            ? static_cast<const BlockT*>(child_ptr)->minData()
            : static_cast<const NodeT*>(child_ptr)->min_data;
        // Only consider children with at least 1 integration.
        if (child_min_data.field.weight > 0) {
            const field_t occupancy = get_field(child_min_data);
            if (occupancy < min_occupancy) {
                min_mean_occupancy = child_min_data.field.occupancy;
                min_weight = child_min_data.field.weight;
                min_occupancy = occupancy;
                min_data_count++;
            }
        }

        const auto& child_max_data = child_ptr->is_block
            ? static_cast<const BlockT*>(child_ptr)->getMaxData()
            : static_cast<const NodeT*>(child_ptr)->max_data;
        // Only consider children with at least 1 integration.
        if (child_max_data.field.weight > 0) {
            const field_t occupancy = get_field(child_max_data);
            if (occupancy > max_occupancy) {
                max_mean_occupancy = child_max_data.field.occupancy;
                max_weight = child_max_data.field.weight;
                max_occupancy = occupancy;
                max_data_count++;
            }
        }
        assert(child_min_data.field.observed == child_max_data.field.observed);
        if (child_max_data.field.observed == true) {
            observed_count++;
        }
    }
    if (min_data_count > 0) {
        node.min_data.field.occupancy = min_mean_occupancy; // TODO: Need to check update?
        node.min_data.field.weight = min_weight;
        if (observed_count == 8) {
            node.min_data.field.observed = true;
        }
    }
    if (max_data_count > 0) {
        node.max_data.field.occupancy = max_mean_occupancy; // TODO: Need to check update?
        node.max_data.field.weight = max_weight;
        if (observed_count == 8) {
            node.max_data.field.observed = true;
        }
    }
    return node.max_data;
}

template<typename BlockT>
void propagate_block_to_coarsest_scale(se::OctantBase* octant_ptr)
{
    assert(octant_ptr);
    assert(octant_ptr->is_block);
    BlockT& block = *static_cast<BlockT*>(octant_ptr);
    if (block.current_scale == block.max_scale) {
        return;
    }
    return propagate_block_to_scale<BlockT>(octant_ptr, BlockT::max_scale);
}

template<typename BlockT>
void propagate_block_to_scale(se::OctantBase* octant_ptr, int desired_scale)
{
    assert(octant_ptr);
    assert(octant_ptr->is_block);

    typedef typename BlockT::DataType DataType;

    BlockT& block = *static_cast<BlockT*>(octant_ptr);

    assert(desired_scale >= block.current_scale);

    int child_scale = block.current_scale;
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    int size_at_parent_scale_li = BlockT::size >> parent_scale;
    int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

    DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
    DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
    DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
    DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

    // Iter over all parent scale data
    for (int z = 0; z < size_at_parent_scale_li; z++) {
        for (int y = 0; y < size_at_parent_scale_li; y++) {
            for (int x = 0; x < size_at_parent_scale_li; x++) {
                const int parent_data_idx =
                    x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                auto& parent_data = data_at_parent_scale[parent_data_idx];
                auto& parent_min_data = min_data_at_parent_scale[parent_data_idx];
                auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                field_t mean_occupancy = 0;
                weight_t mean_weight = 0;

                field_t min_mean_occupancy = 0;
                weight_t min_weight = 0;
                field_t min_occupancy = std::numeric_limits<field_t>::max();

                field_t max_mean_occupancy = 0;
                weight_t max_weight = 0;
                field_t max_occupancy = std::numeric_limits<field_t>::lowest();

                int data_count = 0;
                int observed_count = 0;

                // Access all 8 children per parent
                for (int k = 0; k < 2; k++) {
                    for (int j = 0; j < 2; j++) {
                        for (int i = 0; i < 2; i++) {
                            const int child_data_idx = (2 * x + i)
                                + (2 * y + j) * size_at_child_scale_li
                                + (2 * z + k) * size_at_child_scale_sq;
                            const auto& child_data = data_at_child_scale[child_data_idx];

                            if (child_data.field.weight > 0) {
                                // Update mean
                                data_count++;
                                mean_occupancy += child_data.field.occupancy;
                                mean_weight += child_data.field.weight;

                                const field_t occupancy = get_field(child_data);

                                if (occupancy > max_occupancy) {
                                    // Update max
                                    max_mean_occupancy = child_data.field.occupancy;
                                    max_weight = child_data.field.weight;
                                    max_occupancy = occupancy;
                                }
                                else if (occupancy < min_occupancy) {
                                    min_mean_occupancy = child_data.field.occupancy;
                                    min_weight = child_data.field.weight;
                                    min_occupancy = occupancy;
                                }
                            }
                            if (child_data.field.observed) {
                                observed_count++;
                            }


                        } // i
                    }     // j
                }         // k

                if (data_count > 0) {
                    parent_data.field.occupancy = mean_occupancy / data_count;
                    parent_data.field.weight = ceil((float) mean_weight) / data_count;
                    parent_data.field.observed = false;

                    parent_min_data.field.occupancy = min_mean_occupancy;
                    parent_min_data.field.weight = min_weight;
                    parent_max_data.field.occupancy = max_mean_occupancy;
                    parent_max_data.field.weight = max_weight;
                    if (observed_count == 8) {
                        parent_max_data.field.observed = true;
                        parent_min_data.field.observed = true;
                        //parent_data.field.observed = true;
                        // ToDo: why original SE sets only max data to observed=true?
                    }
                }

            } // x
        }     // y
    }         // z



    // Now propagate from parent scale to maximum block scale
    for (parent_scale += 1; parent_scale <= desired_scale; ++parent_scale) {
        size_at_parent_scale_li = BlockT::size >> parent_scale;
        size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

        child_scale = parent_scale - 1;
        size_at_child_scale_li = BlockT::size >> child_scale;
        size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* min_data_at_child_scale = block.blockMinDataAtScale(child_scale);
        DataType* max_data_at_child_scale = block.blockMaxDataAtScale(child_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_min_data = min_data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    field_t mean_occupancy = 0;
                    weight_t mean_weight = 0;

                    field_t min_mean_occupancy = 0;
                    weight_t min_weight = 0;
                    field_t min_occupancy = std::numeric_limits<field_t>::max();

                    field_t max_mean_occupancy = 0;
                    weight_t max_weight = 0;
                    field_t max_occupancy = std::numeric_limits<field_t>::lowest();

                    int data_count = 0;
                    int observed_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto& child_data = data_at_child_scale[child_data_idx];
                                const auto& child_min_data =
                                    min_data_at_child_scale[child_data_idx];
                                const auto& child_max_data =
                                    max_data_at_child_scale[child_data_idx];

                                if (child_max_data.field.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.field.occupancy;
                                    mean_weight += child_data.field.weight;

                                    const field_t child_max_occupancy = get_field(child_max_data);
                                    if (child_max_occupancy > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_max_data.field.occupancy;
                                        max_weight = child_max_data.field.weight;
                                        max_occupancy = child_max_occupancy;
                                    }
                                    const field_t child_min_occupancy = get_field(child_min_data);
                                    if (child_min_occupancy < min_occupancy) {
                                        // Update min
                                        min_mean_occupancy = child_min_data.field.occupancy;
                                        min_weight = child_min_data.field.weight;
                                        min_occupancy = child_min_occupancy;
                                    }
                                }
                                if (child_max_data.field.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        parent_data.field.occupancy = mean_occupancy / data_count;
                        parent_data.field.weight = ceil((float) mean_weight) / data_count;
                        parent_data.field.observed = false;
                        // block.setData(parent_data_idx,parent_data); ToDo: check but this should not be needed

                        parent_min_data.field.occupancy = min_mean_occupancy;
                        parent_min_data.field.weight = min_weight;
                        parent_max_data.field.occupancy = max_mean_occupancy;
                        parent_max_data.field.weight = max_weight;
                        if (observed_count == 8) {
                            parent_max_data.field.observed = true;
                            parent_min_data.field.observed = true;
                            //parent_data.field.observed = true;
                            // ToDo: why original SE sets only max data to observed=true?
                        }
                    }

                } // x
            }     // y
        }         // z
    }
}

template<typename BlockT>
void propagate_block_down_to_scale(se::OctantBase* octant_ptr, int desired_scale)
{
    assert(octant_ptr);
    assert(octant_ptr->is_block);

    typedef typename BlockT::DataType DataType;

    BlockT& block = *static_cast<BlockT*>(octant_ptr);

    const int last_scale = block.current_scale;

    int current_scale = last_scale;
    while (current_scale > desired_scale) {
        // down propagation
        block.allocateDownTo(current_scale - 1);
        block.current_scale = current_scale - 1;
        assert(block.current_scale == current_scale - 1);
        // Now do the down propagation
        int child_scale = current_scale - 1;
        int size_at_child_scale_li = BlockT::size >> child_scale;
        int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

        int parent_scale = current_scale;
        int size_at_parent_scale_li = BlockT::size >> parent_scale;
        int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        // Iter over all parent scale data
        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
#pragma omp simd
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];

                    // TODO: this should not occur if last integration scale for this block correct
                    if (!parent_data.field.observed) {
                        continue;
                    }

                    // Access all 8 children per parent
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                auto& child_data = data_at_child_scale[child_data_idx];

                                child_data.field.weight = parent_data.field.weight;
                                child_data.field.occupancy = parent_data.field.occupancy;
                                child_data.field.observed = false;
                                // block.setData(child_data_idx,child_data); ToDo: check but this should not be needed

                            } // i
                        }     // j
                    }         // k
                }             // x
            }                 // y
        }                     // z
        current_scale--;
    }
}


} // namespace ray_integrator

} // namespace se

#endif //SE_RAY_INTEGRATOR_CORE_IMPL_HPP
