/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAY_INTEGRATOR_HPP
#define SE_RAY_INTEGRATOR_HPP

#include <se/integrator/ray_integrator_core.hpp>
#include <se/map/octree/propagator.hpp>
#include <set>
#include <unordered_set>

namespace se {

enum class RayState { FreeSpace, Transition, Occupied, Undefined };

template<typename MapT, typename SensorT>
class RayIntegrator {
    public:
    RayIntegrator(MapT& /* map */,
                  const RayMeasurement<SensorT>& /*ray_measurement*/,
                  const timestamp_t /* timestamp */,
                  std::unordered_set<const OctantBase*>* const /*updated_octants = nullptr*/){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
class RayIntegrator<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>,
                    SensorT> {
    public:
    typedef Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::NodeType NodeType;
    typedef typename MapType::BlockType BlockType;

    /**
    * \brief The config file of the single ray carver
    *
    * \param[in] map   The map to allocate the ray in
    */
    struct RayIntegratorConfig {
        RayIntegratorConfig(const MapType& map) :
                sigma_min(map.getDataConfig().field.sigma_min_factor * map.getRes()),
                sigma_max(map.getDataConfig().field.sigma_max_factor * map.getRes()),
                tau_min(map.getDataConfig().field.tau_min_factor * map.getRes()),
                tau_max(map.getDataConfig().field.tau_max_factor * map.getRes())
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
    };


    /**
    * \brief Setup the single ray carver.
    *
    * \param[in]  map                  The reference to the map to be updated.
    * \param[in]  sensor               The sensor model.
    * \param[in]  ray                  The ray to be integrated.
    * \param[in]  T_WS                 The transformation from sensor to world frame.
    * \param[in]  timestamp            The timestamp of the ray to be integrated.
    */
    RayIntegrator(MapType& map,
                  const RayMeasurement<SensorT>& ray_measurement,
                  const timestamp_t timestamp,
                  std::unordered_set<const OctantBase*>* const updated_octants = nullptr);

    /**
     * \brief Reset ray, pose and timestamp for the integrator
     *
     * \param[in] ray           The new ray measurement
     * \param[in] T_WS          The corresponding pose
     * \param[in] timestamp     The ray timestamp
     * \param[in] skip_check    Boolean to decide if we check if ray can be skipped
     *
     * \return False if ray should be skipped. Otherwise true
     */
    bool resetIntegrator(const RayMeasurement<SensorT>& ray_measurement,
                         const timestamp_t timestamp,
                         bool skip_check = false);

    /**
     * \brief Allocate and update along the ray using a step size depending on the chosen resolution.
     * The resolution is chosen based on the angle between neighboring Lidar rays.
     * Up and down-propagation needed for immediate update is done on-the-fly.
     */
    void operator()();

    /**
     * Update Operations
     */

    void propagateToRoot();

    void propagateBlocksToCoarsestScale();

    /**
    * \brief Return a conservative measure of the expected variance of a sensor model inside a voxel
    *        given its position and depth variance.
    *
    * \param[in] ray_step_depth depth of sample measurement along the ray.
    *
    * \return Estimate of the state along the ray (FreeSpace / Occupied / Behind)
    */
    se::RayState computeVariance(const float ray_step_depth);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
    * \brief Recursively decide if to allocate or terminate a node.
    *
    * \note se::Lidar implementation
    *
    * \tparam SensorTDummy
    * \param[in] ray_sample     The current sample point along the ray
    * \param[in] voxel_coord    The voxel coordinates of the current sample along the ray
    * \param[in] rayState       The state along the ray (is it free space?)
    * \param[in] octant_ptr     Starting point for tree traversal
    * \return False if ray-casting can be terminated (e.g. in case a large free-space leaf node is traversed). True otherwise.
    */
    bool operator()(const Eigen::Vector3f& ray_sample,
                    const Eigen::Vector3i& voxel_coord,
                    se::RayState rayState,
                    se::OctantBase* octant_ptr);


    /**
     * Update Operations
     */

    void updateBlock(se::OctantBase* octant_ptr,
                     Eigen::Vector3i& voxel_coords,
                     int desired_scale,
                     float sample_dist);

    MapType& map_;
    OctreeType& octree_;

    std::vector<std::set<se::OctantBase*>> node_set_;
    std::vector<se::OctantBase*> updated_blocks_vector_;
    std::unordered_set<se::OctantBase*>
        updated_blocks_set_; // This set is to keep track of blocks that need to be up-propagated
    std::unordered_set<const se::OctantBase*>* updated_octants_ = nullptr;
    RayIntegratorConfig config_;

    const RayMeasurement<SensorT>* measurement_;
    Eigen::Vector3i last_visited_voxel_;
    Eigen::Isometry3f T_SW_;

    const float map_res_;

    int free_space_scale_ = 0;
    int computed_integration_scale_ = 0;
    timestamp_t timestamp_;

    // Sensor Model - Tau and Sigma
    float ray_dist_ = 0.;
    float tau_ = 0.;
    float three_sigma_ = 0.;
};


template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
class RayIntegrator<Map<Data<se::Field::TSDF, ColB, IdB>, se::Res::Single, BlockSize>, SensorT> {
    public:
    typedef Map<Data<se::Field::TSDF, ColB, IdB>, se::Res::Single, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::NodeType NodeType;
    typedef typename MapType::BlockType BlockType;

    /**
    * \brief The config file of the single ray carver
    *
    * \param[in] map   The map to allocate the ray in
    */
    struct RayIntegratorConfig {
        RayIntegratorConfig(const MapType& map) :
                truncation_boundary(map.getDataConfig().field.truncation_boundary_factor
                                    * map.getRes())
        {
        }

        const float truncation_boundary;
    };


    /**
    * \brief Setup the single ray carver.
    *
    * \param[in]  map                  The reference to the map to be updated.
    * \param[in]  sensor               The sensor model.
    * \param[in]  ray                  The ray to be integrated.
    * \param[in]  T_WS                 The transformation from sensor to world frame.
    * \param[in]  timestamp            The timestamp of the ray to be integrated.
    */
    RayIntegrator(MapType& map,
                  const RayMeasurement<SensorT>& ray_measurement,
                  const timestamp_t timestamp,
                  std::unordered_set<const OctantBase*>* const updated_octants = nullptr);

    /**
     * \brief Reset ray, pose and timestamp for the integrator
     *
     * \param[in] ray           The new ray measurement
     * \param[in] T_WS          The corresponding pose
     * \param[in] timestamp     The ray timestamp
     *
     */
    bool resetIntegrator(const RayMeasurement<SensorT>& ray_measurement,
                         const timestamp_t timestamp);

    /**
     * \brief Allocate and update along the ray using a step size depending on the chosen resolution.
     * The resolution is chosen based on the angle between neighboring Lidar rays.
     * Up and down-propagation needed for immediate update is done on-the-fly.
     */
    bool operator()();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
     * Update Operations
     */

    bool updateBlock(se::OctantBase* block_ptr,
                     Eigen::Vector3i& voxel_coord,
                     const Eigen::Affine3f T_CV,
                     const float measurement_distance);

    MapType& map_;
    OctreeType& octree_;
    std::vector<std::set<se::OctantBase*>> node_set_;
    std::unordered_set<const se::OctantBase*>* updated_octants_ = nullptr;
    RayIntegratorConfig config_;

    const RayMeasurement<SensorT>* measurement_;
    Eigen::Vector3f ray_dir_W_;

    const float map_res_;

    timestamp_t timestamp_;
    float ray_dist_ = 0.;

    //Members to cache if colour or id has to be integrated
    bool has_colour_;
    bool has_id_;
};

} // namespace se
#include "impl/ray_integrator_impl.hpp"

#endif // SE_RAY_INTEGRATOR_HPP
