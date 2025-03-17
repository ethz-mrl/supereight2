/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
#define SE_SINGLERES_TSDF_UPDATER_IMPL_HPP

namespace se {

template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::Updater(
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

    const float truncation_boundary =
        map.getRes() * map.getDataConfig().field.truncation_boundary_factor;
    const Eigen::Isometry3f T_CW = measurements.depth.T_WC.inverse();
    // Transformation from the octree frame V (in voxels) to the sensor frame C (in meters).
    const Eigen::Affine3f T_CV = T_CW * map.getTWM() * Eigen::Scaling(map.getRes())
        * Eigen::Translation3f(sample_offset_frac);

#pragma omp parallel for
    for (size_t i = 0; i < block_ptrs.size(); i++) {
        assert(block_ptrs[i]);
        assert(block_ptrs[i]->is_block);
        auto& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.timestamp = timestamp;
        const Eigen::Vector3i block_coord = block.coord;

        for (int x = 0; x < BlockType::size; ++x) {
            for (int y = 0; y < BlockType::size; ++y) {
                for (int z = 0; z < BlockType::size; ++z) {
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

                    DataType& data = block.getData(voxel_coord);
                    const bool field_updated = data.field.update(
                        sdf_value, truncation_boundary, map.getDataConfig().field.max_weight);

                    // Compute the coordinates of the depth hit in the depth sensor frame C if data
                    // other than depth needs to be integrated.
                    Eigen::Vector3f hit_C;
                    if constexpr (ColB == Colour::On || SemB == Semantics::On) {
                        if (has_colour && field_updated) {
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
                                data.colour.update(
                                    measurements.colour->image(colour_pixel.x(), colour_pixel.y()),
                                    map.getDataConfig().field.max_weight);
                            }
                        }
                    }
                }
            }
        }
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
