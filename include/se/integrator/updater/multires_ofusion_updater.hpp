/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_UPDATER_HPP
#define SE_MULTIRES_OFUSION_UPDATER_HPP

#include <se/integrator/updater/multires_ofusion_core.hpp>
#include <set>

namespace se {



// Multi-res Occupancy updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>, SensorT> {
    public:
    typedef Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;


    struct UpdaterConfig {
        UpdaterConfig(const MapType& map) :
                sigma_min(map.getRes() * map.getDataConfig().field.sigma_min_factor),
                sigma_max(map.getRes() * map.getDataConfig().field.sigma_max_factor),
                tau_min(map.getRes() * map.getDataConfig().field.tau_min_factor),
                tau_max(map.getRes() * map.getDataConfig().field.tau_max_factor)
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
    };

    /**
     * \param[in]  map                  The reference to the map to be updated.
     * \param[in]  sensor               The sensor model.
     * \param[in]  depth_img            The depth image to be integrated.
     * \param[in]  T_WS                 The transformation from sensor to world frame.
     * \param[in]  timestamp            The timestamp of the image to be integrated.
     */
    Updater(MapType& map, const timestamp_t timestamp, const Measurements<SensorT>& measurements);

    void operator()(VolumeCarverAllocation& allocation_list,
                    std::set<const OctantBase*>* const updated_octants = nullptr);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
   . * \brief Propage all newly integrated values from the voxel block depth up to the root of the octree.
   . */
    void propagateToRoot(std::vector<OctantBase*>& block_list);

    void freeBlock(OctantBase* octant_ptr);

    /**
     * \brief Compute integration scale for a given voxel block and update all voxels that project into the image plane.
     *
     * \note The minimum integration scale has only an effect if no data has been integrated into the block yet, i.e.
     *       the integration scale of the block has not been initialised yet.
     *
     * \param[out] block                 The block to be updated.
     * \param[out] min_integration_scale The minimum integration scale.
     */
    void updateBlock(OctantBase* octant_ptr, bool low_variance, bool project_inside);

    /** Update all voxels of \p block with centre \p block_centre_C at \p scale. The template
     * argument \p UpdateBuffer determines whether the update happens on the block buffer or on the
     * actual block data.
     */
    template<bool UpdateBuffer>
    void updateBlockData(BlockType& block,
                         const Eigen::Vector3f& block_centre_C,
                         const int scale,
                         const bool low_variance,
                         const bool project_inside);

    /**
     * \brief Recursively reduce all children by the minimum occupancy log-odd for a single integration.
     */
    void freeNodeRecurse(OctantBase* octant_ptr, int depth);

    private:
    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const Image<float>& depth_img_;
    const Image<float>* const sigma_img_;
    const Eigen::Isometry3f T_CW_;
    const SensorT* const colour_sensor_;
    const Image<colour_t>* const colour_img_;
    Eigen::Isometry3f T_CcC_;
    const bool has_colour_;
    const timestamp_t timestamp_;
    const float map_res_;
    const UpdaterConfig config_;
    std::vector<std::set<OctantBase*>> node_set_;
    std::vector<OctantBase*> freed_block_list_;
    std::set<const OctantBase*>* updated_octants_ = nullptr;
};



} // namespace se

#include "impl/multires_ofusion_updater_impl.hpp"

#endif // SE_MULTIRES_OFUSION_UPDATER_HPP
