/*
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

namespace se {
namespace details {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateDepthImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const timestamp_t timestamp,
                          const Measurements<SensorT>& measurements,
                          std::unordered_set<const OctantBase*>* const updated_octants);
};



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayBatchImplD {
    template<typename SensorT, typename MapT>
    static void integrate(
        MapT& map,
        const SensorT& sensor,
        const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                          Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
            rayPoseBatch,
        const timestamp_t timestamp,
        std::unordered_set<const OctantBase*>* const updated_octants);
};



/**
 * TSDF integration helper struct for partial function specialisation
 */
template<Res ResT>
struct IntegrateDepthImplD<se::Field::TSDF, ResT> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const timestamp_t timestamp,
                          const Measurements<SensorT>& measurements,
                          std::unordered_set<const OctantBase*>* const updated_octants)
    {
        assert(measurements.depth.sensor.model.imageWidth() == measurements.depth.image.width());
        assert(measurements.depth.sensor.model.imageHeight() == measurements.depth.image.height());
        // Allocation
        TICK("allocation")
        se::RaycastCarver raycast_carver(map,
                                         measurements.depth.sensor,
                                         measurements.depth.image,
                                         measurements.depth.T_WC,
                                         timestamp);
        std::vector<OctantBase*> block_ptrs = raycast_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        se::Updater updater(map, block_ptrs, timestamp, measurements);
        TOCK("update")

        if (updated_octants) {
            // TODO: Currently returning allocated, not updated octants. Remove non-updated blocks from
            // block_ptrs during the call to se::Updater::update() to return only updated octants.
            updated_octants->insert(block_ptrs.begin(), block_ptrs.end());
        }
    }
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const timestamp_t timestamp,
                          const Measurements<SensorT>& measurements_,
                          std::unordered_set<const OctantBase*>* const updated_octants)
    {
        // Create a (shallow) copy of the measurements to generate a depth sigma image if needed.
        // The std::optional allows skipping the Image<float> initialization if it's not needed.
        Measurements<SensorT> measurements = measurements_;
        std::optional<Image<float>> depth_sigma;
        if (!measurements.depth_sigma) {
            depth_sigma = uncert::depth_sigma(
                measurements.depth.image, map.getRes(), map.getDataConfig().field);
            measurements.depth_sigma = &depth_sigma.value();
        }
        assert(measurements.depth_sigma);
        assert(measurements.depth.image.width() == measurements.depth.sensor.model.imageWidth());
        assert(measurements.depth.image.height() == measurements.depth.sensor.model.imageHeight());
        assert(measurements.depth.image.width() == measurements.depth_sigma->width());
        assert(measurements.depth.image.height() == measurements.depth_sigma->height());
        // Allocation
        TICK("allocation")
        VolumeCarver<MapT, SensorT> volume_carver(
            map,
            measurements.depth.sensor,
            measurements.depth.image,
            *measurements.depth_sigma,
            measurements.depth.T_WC,
            timestamp); //< process based on variance state and project inside
        se::VolumeCarverAllocation allocation_list = volume_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        se::Updater updater(map, timestamp, measurements);
        updater(allocation_list, updated_octants);
        TOCK("update")
    }
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(
        MapT& map,
        const SensorT& sensor,
        const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                          Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
            rayPoseBatch,
        const timestamp_t timestamp,
        std::unordered_set<const OctantBase*>* const updated_octants)
    {
        se::RayIntegrator<MapT, SensorT> rayIntegrator(
            map, sensor, rayPoseBatch[0].second, rayPoseBatch[0].first, timestamp, updated_octants);

        // do downsampling
        int skip_count = 0;

        for (size_t i = 0; i < rayPoseBatch.size(); i++) {
            TICK("Ray Integration")
            TICK("allocation-integration")
            if (rayIntegrator.resetIntegrator(
                    rayPoseBatch[i].second, rayPoseBatch[i].first, timestamp)) {
                rayIntegrator();
            }
            else {
                skip_count++;
            }
            TOCK("allocation-integration")
            TOCK("Ray Integration")
        }
        // Do Propagations
        TICK("propagateBlocksToCoarsestScale")
        rayIntegrator.propagateBlocksToCoarsestScale();
        TOCK("propagateBlocksToCoarsestScale")
        TICK("propagateToRoot")
        rayIntegrator.propagateToRoot();
        TOCK("propagateToRoot")
    }
};


template<typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayBatchImpl = IntegrateRayBatchImplD<MapT::fld_, MapT::res_>;

} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT& map) : map_(map)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(
    const timestamp_t timestamp,
    const Measurements<SensorT>& measurements,
    std::unordered_set<const OctantBase*>* const updated_octants)
{
    se::details::IntegrateDepthImpl<MapT>::template integrate<SensorT>(
        map_, timestamp, measurements, updated_octants);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRayBatch(
    const timestamp_t timestamp,
    const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                      Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
        rayPoseBatch,
    const SensorT& sensor,
    std::unordered_set<const OctantBase*>* const updated_octants)
{
    se::details::IntegrateRayBatchImpl<MapT>::integrate(
        map_, sensor, rayPoseBatch, timestamp, updated_octants);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
