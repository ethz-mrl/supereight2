/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VISITOR_HPP
#define SE_VISITOR_HPP

#include <optional>
#include <se/map/octree/allocator.hpp>
#include <se/map/octree/fetcher.hpp>

/**
 * Helper wrapper to traverse the octree. All functions take a const octree references and as no manipulation of the octree is done.
 */
namespace se {
namespace visitor {



/// Single/multi-res get data functions

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The data in the voxel to be accessed
 *         Returns init data if block is not allocated
 */
template<typename OctreeT>
typename OctreeT::DataType getData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct block.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] block_ptr   The pointer to the block to checked first
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The data in the voxel to be accessed
 *         Returns init data if block is not allocated
 */
template<typename OctreeT, typename BlockT>
typename OctreeT::DataType
getData(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord);



/// Multi-res get data functions

/**
 * \brief Get the voxel data for a given coordinate and desired scale.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree_ptr     The pointer to the octree
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the data is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The data in octant at the returned scale
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType>
getData(const OctreeT& octree,
        const Eigen::Vector3i& voxel_coord,
        const int scale_desired,
        int& scale_returned);

/**
 * \brief Get the voxel data for a given coordinate and desired scale.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct block.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] block_ptr      The pointer to the block to checked first
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the data is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The data in octant at the returned scale
 */
template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType>
getData(const OctreeT& octree,
        BlockT* block_ptr,
        const Eigen::Vector3i& voxel_coord,
        const int scale_desired,
        int& scale_returned);

/**
 * \brief Get the min occupancy data at a given scale.
 *
 * \tparam OctreeT      The type of octree used (has to be of field type occupancy and multi-res)
 * \param octree        The reference to the octree
 * \param voxel_coord   The voxel coordinates in [voxel] to be accessed
 * \param scale_desired The scale to be accessed
 * \return The min data at the requested scale.
 */
template<typename OctreeT>
inline typename std::enable_if_t<OctreeT::DataType::fld_ == se::Field::Occupancy,
                                 typename OctreeT::DataType>
getMinData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired);

/**
 * \brief Get the max occupancy data at a given scale.
 *
 * \tparam OctreeT      The type of octree used (has to be of field type occupancy and multi-res)
 * \param octree        The reference to the octree
 * \param voxel_coord   The voxel coordinates in [voxel] to be accessed
 * \param scale_desired The scale to be accessed
 * \return The max data at the requested scale.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::DataType::fld_ == Field::Occupancy, typename OctreeT::DataType>
getMaxData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired);

// TODO: Reduce getField functions for single and multi-res to one

/// Single/Multi-res get field functions

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns {}/invalid if the data is invalid.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The field value to be accessed if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT>
std::optional<field_t> getField(const OctreeT& octree, const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns {}/invalid if the data is invalid.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct octant.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] block_ptr   The pointer to the block to checked first
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The field value to be accessed if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT, typename BlockT>
std::optional<field_t>
getField(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord);



/// Multi-res get field functions

/**
 * \brief Get the field value for a given coordinate and desired scale.
 *        The function returns {}/invalid if the data is invalid.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the field value is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The field value at the returned scale if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_t>>
getField(const OctreeT& octree,
         const Eigen::Vector3i& voxel_coord,
         const int scale_desired,
         int& scale_returned);

/**
 * \brief Get the field value for a given coordinate and desired scale.
 *        The function returns {}/invalid if the data is invalid.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct octant.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] block_ptr      The pointer to the block to checked first
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the field value is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The field value at the returned scale if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_t>>
getField(const OctreeT& octree,
         BlockT* block_ptr,
         const Eigen::Vector3i& voxel_coord,
         const int scale_desired,
         int& scale_returned);



/** Interpolate a member of se::Data of \p octree at \p voxel_coord_f in voxels and scale \p
 * desired_scale. The scale the interpolation is performed at may be coarser than \p desired_scale
 * and is written into \p returned_scale.
 *
 * \param[in] octree          The octree whose data will be interpolated.
 * \param[in] voxel_coord_f   The voxel coordinates the data will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[in] valid           A functor with the following declaration, returning whether the
 *                            supplied data is valid and should be used for interpolation:
 *                            \code{.cpp}
 *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
 *                            bool valid(const typename se::Data<FldT, ColB, IdB>& data);
 *                            \endcode
 * \param[in] get             A functor with the following declaration, returning the data of type
 *                            \p T to be interpolated:
 *                            \code{.cpp}
 *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
 *                            T get(const typename se::Data<FldT, ColB, IdB>& data);
 *                            \endcode
 *                            Type \p T must implement the following operators:
 *                            \code{.cpp}
 *                            T operator+(const T& a, const T& b);
 *                            T operator*(const T& a, const float b);
 *                            \endcode
 * \param[in]  desired_scale  The finest scale the data should be interpolated at. Ignored for
 *                            se::Res::Single octrees.
 * \param[out] returned_scale The actual scale the data was interpolated at will be stored into \p
 *                            *returned_scale if \p returned_scale is non-null. \p *returned_scale
 *                            is not modified if \p std::nullopt is returned. The value of \p
 *                            *returned_scale will not be less than \p desired_scale in
 *                            se::Res::Multi octrees.
 * \return The interpolated data if valid, \p std::nullopt otherwise.
 */
template<typename OctreeT, typename ValidF, typename GetF>
std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>
interp(const OctreeT& octree,
       const Eigen::Vector3f& voxel_coord_f,
       ValidF valid,
       GetF get,
       const Scale desired_scale = 0,
       Scale* const returned_scale = nullptr);

/** Interpolate the field of \p octree at \p voxel_coord_f and \p desired_scale. See
 * se::visitor::interp() for more detailed documentation.
 */
template<typename OctreeT>
std::optional<field_t> interpField(const OctreeT& octree,
                                   const Eigen::Vector3f& voxel_coord_f,
                                   const Scale desired_scale = 0,
                                   Scale* const returned_scale = nullptr);

/** Interpolate the colour of \p octree at \p voxel_coord_f and \p desired_scale. See
 * se::visitor::interp() for more detailed documentation.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::col_ == Colour::On, std::optional<colour_t>>
interpColour(const OctreeT& octree,
             const Eigen::Vector3f& voxel_coord_f,
             const Scale desired_scale = 0,
             Scale* const returned_scale = nullptr);



/** Return the gradient of a member of se::Data of \p octree at \p voxel_coord_f in voxels and scale
 * \p desired_scale. The scale the gradient is computed at may be coarser than \p desired_scale and
 * is written into \p returned_scale.
 *
 * \param[in] octree          The octree whose data will be used for the gradient computation the data.
 * \param[in] voxel_coord_f   The voxel coordinates the data gradient will be computed at. The
 *                            coordinates may have a fractional part.
 * \param[in] valid           A functor with the following declaration, returning whether the
 *                            supplied data is valid and should be used for gradient computation:
 *                            \code{.cpp}
 *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
 *                            bool valid(const typename se::Data<FldT, ColB, IdB>& data);
 *                            \endcode
 * \param[in] get             A functor with the following declaration, returning the data of type
 *                            \p T the gradient of which will be computed:
 *                            \code{.cpp}
 *                            template<se::Field FldT, se::Colour ColB, se::Id IdB>
 *                            T get(const typename se::Data<FldT, ColB, IdB>& data);
 *                            \endcode
 *                            Type \p T must implement the following operators:
 *                            \code{.cpp}
 *                            T operator+(const T& a, const T& b);
 *                            T operator-(const T& a, const T& b);
 *                            T operator*(const T& a, const float b);
 *                            \endcode
 * \param[in]  desired_scale  The finest scale the data gradient should be computed at. Ignored for
 *                            se::Res::Single octrees.
 * \param[out] returned_scale The actual scale the gradient was computed at will be stored into \p
 *                            *returned_scale if \p returned_scale is non-null. \p *returned_scale
 *                            is not modified if \p std::nullopt is returned. The value of \p
 *                            *returned_scale will not be less than \p desired_scale in
 *                            se::Res::Multi octrees.
 * \return The data gradient if valid, \p std::nullopt otherwise.
 */
template<typename OctreeT, typename ValidF, typename GetF>
std::optional<Eigen::Matrix<std::invoke_result_t<GetF, typename OctreeT::DataType>, 3, 1>>
grad(const OctreeT& octree,
     const Eigen::Vector3f& voxel_coord_f,
     ValidF valid,
     GetF get,
     const Scale desired_scale = 0,
     Scale* const returned_scale = nullptr);

/** Return the field gradient of \p octree at \p voxel_coord_f and \p desired_scale. See
 * se::visitor::grad() for more detailed documentation.
 */
template<typename OctreeT>
std::optional<field_vec_t> gradField(const OctreeT& octree,
                                     const Eigen::Vector3f& voxel_coord_f,
                                     const Scale desired_scale = 0,
                                     Scale* const returned_scale = nullptr);

} // namespace visitor
} // namespace se

#include "impl/visitor_impl.hpp"

#endif // SE_VISITOR_HPP
