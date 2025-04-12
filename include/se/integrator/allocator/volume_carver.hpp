/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VOLUME_CARVER_HPP
#define SE_VOLUME_CARVER_HPP

#include <Eigen/Geometry>
#include <se/integrator/allocator/dense_pooling_image.hpp>
#include <se/map/map.hpp>

namespace se {



enum class VarianceState { Constant, Gradient, Undefined };

struct VolumeCarverAllocation {
    std::vector<se::OctantBase*> node_list;
    std::vector<se::OctantBase*> block_list;
    std::vector<se::VarianceState> variance_state_list;
    std::vector<bool> projects_inside_list;
};



/** Unimplemented on purpose so that the template specialization is used. */
template<typename MapT, typename SensorT>
class VolumeCarver {
    public:
    VolumeCarver(MapT& /* map */,
                 const SensorT& /* sensor */,
                 const se::Image<float>& /* depth_img */,
                 const Eigen::Isometry3f& /* T_WS */,
                 const timestamp_t /* timestamp */);
};



/**
 * \brief Allocate the frustum using a map-to-camera volume carving approach
 *
 * \tparam ColB
 * \tparam IdB
 * \tparam BlockSize
 * \tparam SensorT
 */
template<se::Colour ColB, se::Id IdB, int BlockSize, typename SensorT>
class VolumeCarver<Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize>, SensorT> {
    public:
    typedef Map<Data<se::Field::Occupancy, ColB, IdB>, se::Res::Multi, BlockSize> MapType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename OctreeType::NodeType NodeType;
    typedef typename OctreeType::BlockType BlockType;

    /**
     * \brief The config file of the volume carver
     *
     * \param[in] map   The map allocate the frustum in
     */
    struct VolumeCarverConfig {
        VolumeCarverConfig(const MapType& map) :
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
     * \brief Setup the volume carver.
     *
     * \param[in]  map                The reference to the map to be updated.
     * \param[in]  sensor             The sensor model.
     * \param[in]  depth_img          The depth image to be integrated.
     * \param[in]  depth_sigma_img    The uncertainty (standard deviation) image of depth_img
     * \param[in]  T_WS               The transformation from sensor to world frame.
     * \param[in]  timestamp          The timestamp of the frame to be integrated. Currently unused.
     */
    VolumeCarver(MapType& map,
                 const SensorT& sensor,
                 const se::Image<float>& depth_img,
                 const se::Image<float>& depth_sigma_img,
                 const Eigen::Isometry3f& T_WS,
                 const timestamp_t timestamp);

    /**
     * \brief Allocate the frustum using a map-to-camera volume carving approach
     */
    VolumeCarverAllocation operator()();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



    private:
    /**
     * \brief Verify if the node crosses the camera frustum excluding the case of the camera in the node.
     *
     * \param[in] proj_corner_stati The stati of the projection of the eight octant corners into the image plane
     *
     * \return True/false statement if node crosses the camera frustum.
     */
    bool crossesFrustum(std::vector<srl::projection::ProjectionStatus>& proj_corner_stati);

    /**
     * \brief Verify if the camera is inside a given node.
     *
     * \param[in] node_coord  The coordinates of the node in voxel coordinates.
     * \param[in] node_size   The size of the node in voxel units.
     * \param[in] T_WS        The transformation from sensor to world frame.
     *
     * \return True/false statement if the camera is inside the node.
     */
    bool cameraInNode(const Eigen::Vector3i& node_coord,
                      const int node_size,
                      const Eigen::Isometry3f& T_WS);

    /**
     * \brief Return a conservative meassure of the expected variance of a sensor model inside a voxel
     *        given its position and depth variance.
     *
     * \param[in] depth_value_min Depth measurement max value inside voxel.
     * \param[in] depth_value_max Depth measurement min value inside voxel.
     * \param[in] node_dist_min_m Minimum node distance along z-axis in meter.
     * \param[in] node_dist_max_m Maximum node distance along z-axis in meter.
     * \param[in] std_dev_max     Maximum standard deviation of a measurement.
     *
     * \return Estimate of the variance
     */
    se::VarianceState computeVariance(const float depth_value_min,
                                      const float depth_value_max,
                                      const float node_dist_min_m,
                                      const float node_dist_max_m,
                                      const float std_dev_max);

    /**
     * \brief Recursively decide if to allocate or terminate a node.
     *
     * \note se::PinholeCamera implementation
     *
     * \tparam SensorTDummy
     * \param[in] node_coord      The coordinates of the octant corner (front, left, bottom) to be evaluated
     * \param[in] node_size       The size of the octant in [voxel]
     * \param[in] octant_depth    The tree depth of the octant
     * \param[in] parent_ptr      The pointer to the octant
     */
    template<class SensorTDummy = SensorT>
    typename std::enable_if_t<std::is_same<SensorTDummy, se::PinholeCamera>::value, void>
    operator()(const Eigen::Vector3i& octant_coord,
               const int octant_size,
               const int octant_depth,
               se::OctantBase* octant_ptr);

    /**
     * \brief Recursively decide if to allocate or terminate a node.
     *
     * \note se::OusterLidar implementation
     *
     * \tparam SensorTDummy
     * \param[in] node_coord      The coordinates of the octant corner (front, left, bottom) to be evaluated
     * \param[in] node_size       The size of the octant in [voxel]
     * \param[in] octant_depth    The tree depth of the octant
     * \param[in] parent_ptr      The pointer to the octant
     */
    template<class SensorTDummy = SensorT>
    typename std::enable_if_t<std::is_same<SensorTDummy, se::OusterLidar>::value, void>
    operator()(const Eigen::Vector3i& octant_coord,
               const int octant_size,
               const int octant_depth,
               se::OctantBase* octant_ptr);

    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const se::DensePoolingImage<SensorT> depth_pooling_img_;
    // XXX: the sigma pooling image isn't strictly necessary, we only need the sigma corresponding
    // to the maximum depth at each depth pooling image lookup. Find a way to avoid it or combine
    // both pooling images into one. One way to avoid it would be to also store the pixel
    // coordinates corresponding to se::Pixel::min and se::Pixel::max in se::Pixel. Then, getting
    // the sigma corresponding to the maximum depth would be just an image lookup.
    const se::DensePoolingImage<SensorT> sigma_pooling_img_;
    const Eigen::Isometry3f T_SW_;
    const float map_res_;
    VolumeCarverConfig config_;
    const float max_depth_value_;
    const float zero_depth_band_;
    const float size_to_radius_;
    VolumeCarverAllocation allocation_list_;
};



} // namespace se

#include "impl/volume_carver_impl.hpp"

#endif // SE_VOLUME_CARVER_HPP
