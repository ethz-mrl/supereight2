/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include <se/map/octant/octant.hpp>

namespace se {
namespace fetcher {

/** Unit-less relative offsets to the 6 face neighbours of an octant. */
// clang-format off
static const Eigen::Matrix<int, 3, 6> face_neighbour_offsets = (Eigen::Matrix<int, 3, 6>() <<
     0,  0, -1,  1,  0,  0,
     0, -1,  0,  0,  1,  0,
    -1,  0,  0,  0,  0,  1).finished();
// clang-format on



/** Return the octant with coordinates in voxels \p octant_coord and scale \p scale_desired.
 *
 * \tparam OctreeT        The type of the se::Octree \p base_parent_ptr is part of.
 * \param octant_coord    The coordinates in voxels of the octant to be fetched.
 * \param scale_desired   The scale of the octant to be fetched.
 * \param base_parent_ptr The octant to start searching from (e.g. the octree root).
 * \return The pointer to the fetched octant or nullptr if no octant is allocated at the supplied
 * coordinates and scale.
 */
template<typename OctreeT>
OctantBase* octant(const Eigen::Vector3i& octant_coord,
                   const scale_t scale_desired,
                   OctantBase* const base_parent_ptr);

/** Same as se::fetcher::octant() but returning a pointer to const.
 */
template<typename OctreeT>
const OctantBase* octant(const Eigen::Vector3i& octant_coord,
                         const scale_t scale_desired,
                         const OctantBase* const base_parent_ptr);

/** Return the finest allocated octant with coordinates in voxels \p octant_coord and scale up to \p
 * scale_desired.
 *
 * \tparam OctreeT        The type of the se::Octree \p base_parent_ptr is part of.
 * \param octant_coord    The coordinates in voxels of the octant to be fetched.
 * \param scale_desired   The maximum scale of the octant to be fetched.
 * \param base_parent_ptr The octant to start searching from, e.g. the octree root.
 * \return The pointer to the fetched octant or nullptr if no octant is allocated at the supplied
 * coordinates.
 */
template<typename OctreeT>
OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                          const scale_t scale_desired,
                          OctantBase* const base_parent_ptr);

/** Same as se::fetcher::finest_octant() but returning a pointer to const.
 */
template<typename OctreeT>
const OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                const scale_t scale_desired,
                                const OctantBase* const base_parent_ptr);

/** Return the block with coordinates in voxels \p block_coord.
 *
 * \tparam OctreeT        The type of the se::Octree \p base_parent_ptr is part of.
 * \param block_coord     The coordinates in voxels of the block to be fetched.
 * \param base_parent_ptr The octant to start searching from, e.g. the octree root.
 * \return The pointer to the fetched block or nullptr if no block is allocated at the supplied
 * coordinates.
 */
template<typename OctreeT>
OctantBase* block(const Eigen::Vector3i& block_coord, OctantBase* const base_parent_ptr);

/** Same as se::fetcher::block() but returning a pointer to const.
 */
template<typename OctreeT>
const OctantBase* block(const Eigen::Vector3i& block_coord,
                        const OctantBase* const base_parent_ptr);

/** Return the finest allocated octant with coordinates in voxels \p block_coord.
 *
 * \tparam OctreeT        The type of the se::Octree \p base_parent_ptr is part of.
 * \param block_coord     The coordinates in voxels of the octant to be fetched.
 * \param base_parent_ptr The octant to start searching from, e.g. the octree root.
 * \return The pointer to the fetched octant or nullptr if no octant is allocated at the supplied
 * coordinates.
 */
template<typename OctreeT>
OctantBase* leaf(const Eigen::Vector3i& leaf_coord, OctantBase* const base_parent_ptr);

/** Same as se::fetcher::leaf() but returning a pointer to const.
 */
template<typename OctreeT>
const OctantBase* leaf(const Eigen::Vector3i& leaf_coord, const OctantBase* const base_parent_ptr);

/** Return the face neighbours of \p octant_ptr which is an octant of \p octree. The returned face
 * neighbours will be at the same or higher scale than \p octant_ptr. A nullptr is returned for each
 * unallocated face neighbour. Face neighbours outside the octree are ignored. Octants at the faces,
 * edges and corners of the octree volume have 5, 4 and 3 face neighbours respectively, thus the
 * function will return between 3 and 6, potentially null, pointers to neighbours.
 */
template<typename OctreeT>
std::vector<const OctantBase*> face_neighbours(const OctantBase* const octant_ptr,
                                               const OctreeT& octree);

} // namespace fetcher
} // namespace se

#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
