/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
#define SE_MULTIRES_TSDF_UPDATER_IMPL_HPP

namespace se {

// Multi-res TSDF updater
template<Colour ColB, Id IdB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::TSDF, ColB, IdB>, Res::Multi, BlockSize>, SensorT>::Updater(
    MapType& map,
    std::vector<OctantBase*>& block_ptrs,
    const timestamp_t timestamp,
    const Measurements<SensorT>& measurements)
{
    const bool has_colour = ColB == Colour::On && measurements.colour;
    Eigen::Isometry3f T_CcC;
    if constexpr (ColB == Colour::On) {
        if (has_colour) {
            T_CcC = measurements.colour->T_WC.inverse() * measurements.depth.T_WC;
        }
    }

    const bool has_id = IdB == Id::On && measurements.ids;
    Eigen::Isometry3f T_CidC;
    if constexpr (IdB == Id::On) {
        if (has_id) {
            T_CidC = measurements.ids->T_WC.inverse() * measurements.depth.T_WC;
        }
    }


    const float truncation_boundary =
        map.getRes() * map.getDataConfig().field.truncation_boundary_factor;
    const Eigen::Isometry3f T_CW = measurements.depth.T_WC.inverse();
    // Transformation from the octree frame V (in voxels) to the sensor frame C (in meters).
    const Eigen::Affine3f T_CV = T_CW * map.getTWM() * Eigen::Scaling(map.getRes())
        * Eigen::Translation3f(sample_offset_frac);
    const Octree<Data<Field::TSDF, ColB, IdB>, Res::Multi, BlockSize>& octree = map.getOctree();

#pragma omp parallel for
    for (size_t i = 0; i < block_ptrs.size(); i++) {
        assert(block_ptrs[i]);
        assert(block_ptrs[i]->is_block);
        auto& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.timestamp = timestamp;
        const Eigen::Vector3i block_coord = block.coord;
        Eigen::Vector3f block_centre_point_W;
        map.voxelToPoint(block_coord, block.size, block_centre_point_W);
        const Eigen::Vector3f block_centre_point_C = T_CW * block_centre_point_W;
        const int last_curr_scale = block.current_scale;
        const int lower_curr_scale_limit = last_curr_scale - 1;

        const int curr_scale =
            std::max(measurements.depth.sensor.computeIntegrationScale(block_centre_point_C,
                                                                       map.getRes(),
                                                                       last_curr_scale,
                                                                       block.min_scale,
                                                                       block.max_scale),
                     lower_curr_scale_limit);

        block.min_scale = block.min_scale < 0 ? curr_scale : std::min(block.min_scale, curr_scale);

        // Down-propagate the block data to the new, finer scale.
        if (curr_scale < last_curr_scale) {
            // Reset the parent delta data after it has been down-propagated to all its children.
            auto parent_down_funct = [](const OctreeType& /* octree */,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& data_union) {
                data_union.past_data.field.tsdf = data_union.data.field.tsdf;
                data_union.past_data.field.weight = 0;
                if constexpr (ColB == Colour::On) {
                    data_union.past_data.colour.colour = data_union.data.colour.colour;
                    data_union.past_data.colour.weight = 0;
                }
                if constexpr (IdB == Id::On) {
                    data_union.past_data.id.id = data_union.data.id.id;
                }
            };

            // Update or initialize the child data using the parent data.
            auto child_down_funct = [&](const OctreeType& octree,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& child_data_union,
                                        const typename BlockType::DataUnion& parent_data_union) {
                if (is_valid(child_data_union.data)) {
                    // Perform a delta update on the child using the parent data.
                    const field_t delta_tsdf =
                        parent_data_union.data.field.tsdf - parent_data_union.past_data.field.tsdf;
                    child_data_union.data.field.tsdf =
                        std::max(child_data_union.data.field.tsdf + delta_tsdf, field_t(-1));
                    child_data_union.data.field.weight =
                        fminf(child_data_union.data.field.weight
                                  + parent_data_union.past_data.field.weight,
                              map.getDataConfig().field.max_weight);
                    child_data_union.past_data.field.weight =
                        parent_data_union.past_data.field.weight;
                }
                else {
                    // This child hasn't been observed before, initialize to the interpolated field
                    // value.
                    const int child_size = octantops::scale_to_size(child_data_union.scale);
                    const Eigen::Vector3f child_sample_coord_f =
                        child_data_union.coord.template cast<float>()
                        + sample_offset_frac * child_size;
                    const auto interp_field_value = visitor::getFieldInterp(
                        octree, child_sample_coord_f, child_data_union.scale);
                    if (interp_field_value) {
                        child_data_union.data.field.tsdf = *interp_field_value;
                        child_data_union.data.field.weight = parent_data_union.data.field.weight;
                        child_data_union.past_data.field.tsdf = child_data_union.data.field.tsdf;
                        child_data_union.past_data.field.weight = 0;
                        if constexpr (ColB == Colour::On) {
                            const auto interp_colour_value = visitor::getColourInterp(
                                octree, child_sample_coord_f, child_data_union.scale);
                            // Colour interpolation may fail even if field interpolation succeeded
                            // as it requires both field and colour data to be valid.
                            if (interp_colour_value) {
                                child_data_union.data.colour.colour = *interp_colour_value;
                                child_data_union.data.colour.weight =
                                    parent_data_union.data.colour.weight;
                                child_data_union.past_data.colour.colour =
                                    child_data_union.data.colour.colour;
                                child_data_union.past_data.colour.weight = 0;
                            }
                        }
                        if constexpr (IdB == Id::On) {
                            child_data_union.data.id.id = parent_data_union.data.id.id;
                            child_data_union.past_data.id.id = child_data_union.data.id.id;
                        }
                    }
                }
            };

            propagator::propagateBlockDown(octree,
                                           static_cast<OctantBase*>(&block),
                                           curr_scale,
                                           child_down_funct,
                                           parent_down_funct);
        }

        block.current_scale = curr_scale;
        const int stride = octantops::scale_to_size(curr_scale);

        for (int x = 0; x < BlockType::size; x += stride) {
            for (int y = 0; y < BlockType::size; y += stride) {
                for (int z = 0; z < BlockType::size; z += stride) {
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                    // Compute the coordinates of the voxel sample position in the sensor frame.
                    const Eigen::Vector3f point_C = T_CV * voxel_coord.cast<float>();

                    if (point_C.norm() > measurements.depth.sensor.farDist(point_C)) {
                        continue;
                    }

                    // Get the depth value this voxel projects into.
                    Eigen::Vector2f depth_pixel_f;
                    if (measurements.depth.sensor.model.project(point_C, &depth_pixel_f)
                        != srl::projection::ProjectionStatus::Successful) {
                        continue;
                    }
                    const Eigen::Vector2i depth_pixel = se::round_pixel(depth_pixel_f);
                    const float depth_value =
                        measurements.depth.image(depth_pixel.x(), depth_pixel.y());
                    if (depth_value < measurements.depth.sensor.near_plane) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = measurements.depth.sensor.measurementFromPoint(point_C);
                    const field_t sdf_value = (depth_value - m) / m * point_C.norm();

                    typename BlockType::DataUnion data_union =
                        block.dataUnion(voxel_coord, block.current_scale);
                    const bool field_updated = data_union.data.field.update(
                        sdf_value, truncation_boundary, map.getDataConfig().field.max_weight);
                    data_union.past_data.field.weight++;

                    // Compute the coordinates of the depth hit in the depth sensor frame C if data
                    // other than depth needs to be integrated.
                    Eigen::Vector3f hit_C;
                    if constexpr (ColB == Colour::On || IdB == Id::On) {
                        if ((has_colour || has_id) && field_updated) {
                            measurements.depth.sensor.model.backProject(depth_pixel_f, &hit_C);
                            hit_C.array() *= depth_value;
                        }
                    }

                    // Update the colour data if possible and only if the field was updated, that is
                    // if we have corresponding depth information.
                    if constexpr (ColB == Colour::On) {
                        if (has_colour && field_updated) {
                            // Project the depth hit onto the colour image.
                            const Eigen::Vector3f hit_Cc = T_CcC * hit_C;
                            Eigen::Vector2f colour_pixel_f;
                            if (measurements.colour->sensor.model.project(hit_Cc, &colour_pixel_f)
                                == srl::projection::ProjectionStatus::Successful) {
                                const Eigen::Vector2i colour_pixel =
                                    se::round_pixel(colour_pixel_f);
                                data_union.data.colour.update(
                                    measurements.colour->image(colour_pixel.x(), colour_pixel.y()),
                                    map.getDataConfig().field.max_weight);
                            }
                        }
                    }

                    // Update the identifier data if possible and only if the field was updated, that
                    // is if we have corresponding depth information.
                    if constexpr (IdB == Id::On) {
                        if (has_id && field_updated) {
                            // Project the depth hit onto the identifier image.
                            const Eigen::Vector3f hit_Cid = T_CidC * hit_C;
                            Eigen::Vector2f id_pixel_f;
                            if (measurements.ids->sensor.model.project(hit_Cid, &id_pixel_f)
                                == srl::projection::ProjectionStatus::Successful) {
                                const Eigen::Vector2i id_pixel = se::round_pixel(id_pixel_f);
                                data_union.data.id.update(
                                    measurements.ids->image(id_pixel.x(), id_pixel.y()));
                            }
                        }
                    }
                }
            }
        }

        // Up-propagate the block data to the coarser scales by setting the parent data to the mean
        // of the valid child data.
        auto aggregate_children_funct =
            [](typename BlockType::DataUnion& parent_data_union,
               const std::array<typename BlockType::DataType, 8>& child_data) {
                const int valid_children = data::up_prop_mean(parent_data_union.data, child_data);
                if (valid_children > 0) {
                    parent_data_union.past_data.field.tsdf = parent_data_union.data.field.tsdf;
                    parent_data_union.past_data.field.weight = 0;
                }
                else {
                    parent_data_union.past_data = typename BlockType::PastDataType();
                }
            };
        propagator::propagateBlockUp(octree, &block, curr_scale, aggregate_children_funct);
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
