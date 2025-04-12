/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_CORE_IMPL_HPP
#define SE_MULTIRES_OFUSION_CORE_IMPL_HPP



namespace se {



template<typename ConfigT>
float compute_three_sigma(const field_t depth_value,
                          const float sigma_min,
                          const float sigma_max,
                          const ConfigT config)
{
    if (config.field.uncertainty_model == UncertaintyModel::Linear) {
        return 3 * std::clamp(config.field.k_sigma * depth_value, sigma_min, sigma_max);
    }
    else {
        return 3 * std::clamp(config.field.k_sigma * math::sq(depth_value), sigma_min, sigma_max);
    }
}



template<typename ConfigT>
float compute_tau(const field_t depth_value,
                  const float tau_min,
                  const float tau_max,
                  const ConfigT config)
{
    return std::clamp(config.field.k_tau * depth_value, tau_min, tau_max);
}



namespace updater {

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
    // Don't update colour in free space.
    // Reset the segment ID to keep the map consistent in case something went from occupied to free.
    if constexpr (DataT::id_ == Id::On) {
        voxel_data.id = typename DataT::IdType();
    }
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

    // Gather the child minimum and maximum data.
    std::array<typename NodeT::DataType, 8> child_min_data;
    std::array<typename NodeT::DataType, 8> child_max_data;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        const OctantBase* const child_ptr = node.getChild(child_idx);
        if (child_ptr) {
            child_min_data[child_idx] = child_ptr->is_block
                ? static_cast<const BlockT*>(child_ptr)->minData()
                : static_cast<const NodeT*>(child_ptr)->min_data;
            child_max_data[child_idx] = child_ptr->is_block
                ? static_cast<const BlockT*>(child_ptr)->maxData()
                : static_cast<const NodeT*>(child_ptr)->max_data;
        }
        else {
            set_invalid(child_min_data[child_idx]);
            set_invalid(child_max_data[child_idx]);
        }
    }

    data::up_prop_min(node.min_data, child_min_data);
    data::up_prop_max(node.max_data, child_max_data);
    return node.max_data;
}



template<typename BlockT>
void propagate_block_to_coarsest_scale(OctantBase* octant_ptr)
{
    assert(octant_ptr);
    assert(octant_ptr->is_block);

    typedef typename BlockT::DataType DataType;

    BlockT& block = *static_cast<BlockT*>(octant_ptr);
    if (block.current_scale == block.max_scale) {
        return;
    }

    int child_scale = block.current_scale;
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    int size_at_parent_scale_li = BlockT::size >> parent_scale;
    int size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

    if (block.buffer_scale() > block.current_scale) {
        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* min_data_at_child_scale = block.blockDataAtScale(child_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* max_data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_max_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_min_data = min_data_at_parent_scale[parent_max_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_max_data_idx];

                    // Gather the child minimum and maximum data.
                    std::array<typename BlockT::DataType, 8> child_min_data;
                    std::array<typename BlockT::DataType, 8> child_max_data;
                    int child_idx = 0;
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                child_min_data[child_idx] = min_data_at_child_scale[child_data_idx];
                                child_max_data[child_idx] = max_data_at_child_scale[child_data_idx];
                                child_idx++;
                            } // i
                        } // j
                    } // k

                    data::up_prop_min(parent_min_data, child_min_data);
                    data::up_prop_max(parent_max_data, child_max_data);
                } // x
            } // y
        } // z
    }
    else {
        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_min_data = min_data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    // Gather the child data.
                    std::array<typename BlockT::DataType, 8> child_data;
                    int child_idx = 0;
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                child_data[child_idx] = data_at_child_scale[child_data_idx];
                                child_idx++;
                            } // i
                        } // j
                    } // k

                    data::up_prop_mean(parent_data, child_data);
                    data::up_prop_min(parent_min_data, child_data);
                    data::up_prop_max(parent_max_data, child_data);
                } // x
            } // y
        } // z
    }

    for (parent_scale += 1; parent_scale <= BlockT::max_scale; ++parent_scale) {
        size_at_parent_scale_li = BlockT::size >> parent_scale;
        size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

        child_scale = parent_scale - 1;
        size_at_child_scale_li = BlockT::size >> child_scale;
        size_at_child_scale_sq = math::sq(size_at_child_scale_li);

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

                    // Gather the child data.
                    std::array<typename BlockT::DataType, 8> child_data;
                    std::array<typename BlockT::DataType, 8> child_min_data;
                    std::array<typename BlockT::DataType, 8> child_max_data;
                    int child_idx = 0;
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                child_data[child_idx] = data_at_child_scale[child_data_idx];
                                child_min_data[child_idx] = min_data_at_child_scale[child_data_idx];
                                child_max_data[child_idx] = max_data_at_child_scale[child_data_idx];
                                child_idx++;
                            } // i
                        } // j
                    } // k

                    data::up_prop_mean(parent_data, child_data);
                    data::up_prop_min(parent_min_data, child_min_data);
                    data::up_prop_max(parent_max_data, child_max_data);
                } // x
            } // y
        } // z
    }
}



} // namespace updater
} // namespace se

#endif // SE_MULTIRES_OFUSION_CORE_IMPL_HPP
