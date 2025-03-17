/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_FETCHER_IMPL_HPP
#define SE_FETCHER_IMPL_HPP



namespace se {
namespace fetcher {



template<typename OctreeT>
OctantBase* octant(const Eigen::Vector3i& octant_coord,
                   const scale_t scale_desired,
                   OctantBase* const base_parent_ptr)
{
    return const_cast<OctantBase*>(octant<OctreeT>(
        octant_coord, scale_desired, static_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
const OctantBase* octant(const Eigen::Vector3i& octant_coord,
                         const scale_t scale_desired,
                         const OctantBase* const base_parent_ptr)
{
    const typename OctreeT::NodeType* parent_ptr =
        static_cast<const typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->size >> 1;
    const OctantBase* child_ptr = nullptr;

    int size_desired = std::max(octantops::scale_to_size(scale_desired),
                                OctreeT::BlockType::size); // Not smaller than block size
    for (; child_size >= size_desired; child_size = child_size >> 1) {
        idx_t child_idx = ((octant_coord.x() & child_size) > 0)
            + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);
        if (!child_ptr) {
            return nullptr;
        }
        parent_ptr = static_cast<const typename OctreeT::NodeType*>(child_ptr);
    }

    return child_ptr;
}



template<typename OctreeT>
OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                          const scale_t scale_desired,
                          OctantBase* const base_parent_ptr)
{
    return const_cast<OctantBase*>(finest_octant<OctreeT>(
        octant_coord, scale_desired, static_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
const OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                const scale_t scale_desired,
                                const OctantBase* const base_parent_ptr)
{
    const typename OctreeT::NodeType* parent_ptr =
        static_cast<const typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->size >> 1;
    const OctantBase* child_ptr = nullptr;

    int size_desired = std::max(octantops::scale_to_size(scale_desired),
                                OctreeT::BlockType::size); // Not smaller than block size
    for (; child_size >= size_desired; child_size = child_size >> 1) {
        idx_t child_idx = ((octant_coord.x() & child_size) > 0)
            + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);

        if (!child_ptr) {
            const OctantBase* leaf_ptr = (parent_ptr->isLeaf()) ? parent_ptr : nullptr;
            return leaf_ptr; // leaf is either a block or a parent with no children!
        }

        parent_ptr = static_cast<const typename OctreeT::NodeType*>(child_ptr);
    }

    return child_ptr;
}



template<typename OctreeT>
OctantBase* block(const Eigen::Vector3i& block_coord, OctantBase* const base_parent_ptr)
{
    return const_cast<OctantBase*>(
        block<OctreeT>(block_coord, static_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
const OctantBase* block(const Eigen::Vector3i& block_coord, const OctantBase* const base_parent_ptr)
{
    const typename OctreeT::NodeType* parent_ptr =
        static_cast<const typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->size >> 1;
    const OctantBase* child_ptr = nullptr;

    for (; child_size >= OctreeT::BlockType::size; child_size = child_size >> 1) {
        idx_t child_idx = ((block_coord.x() & child_size) > 0)
            + 2 * ((block_coord.y() & child_size) > 0) + 4 * ((block_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);
        if (!child_ptr) {
            return nullptr;
        }
        parent_ptr = static_cast<const typename OctreeT::NodeType*>(child_ptr);
    }

    return child_ptr;
}



template<typename OctreeT>
OctantBase* leaf(const Eigen::Vector3i& leaf_coord, OctantBase* const base_parent_ptr)
{
    return const_cast<OctantBase*>(
        leaf<OctreeT>(leaf_coord, static_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
const OctantBase* leaf(const Eigen::Vector3i& leaf_coord, const OctantBase* const base_parent_ptr)
{
    // The finest possible leaves are at the block scale.
    return finest_octant<OctreeT>(leaf_coord, OctreeT::max_block_scale, base_parent_ptr);
}



template<typename OctreeT>
std::vector<const OctantBase*> face_neighbours(const OctantBase* const octant_ptr,
                                               const OctreeT& octree)
{
    assert(octant_ptr);
    const Eigen::Vector3i& octant_coord = octant_ptr->coord;
    const int octant_size = octant_ptr->is_block
        ? static_cast<const typename OctreeT::BlockType*>(octant_ptr)->size
        : static_cast<const typename OctreeT::NodeType*>(octant_ptr)->size;
    const int octant_scale = octantops::size_to_scale(octant_size);
    const Eigen::Matrix<int, 3, 6> neighbour_coords =
        (octant_size * face_neighbour_offsets).colwise() + octant_coord;
    std::vector<const OctantBase*> neighbours;
    for (int i = 0; i < neighbour_coords.cols(); i++) {
        const Eigen::Vector3i& neighbour_coord = neighbour_coords.col(i);
        if (!octree.contains(neighbour_coord)) {
            continue;
        }
        const OctantBase* neighbour =
            finest_octant<OctreeT>(neighbour_coord, octant_scale, octree.getRoot());
        if (neighbour) {
            // If the returned neighbour contains the octant it means that no neighbouring octant
            // has been allocated.
            const Eigen::Vector3i& nc = neighbour->coord;
            if ((octant_coord.array() >= nc.array()).all()
                && (octant_coord.array() < nc.array() + octant_size).all()) {
                neighbour = nullptr;
            }
        }
        neighbours.push_back(neighbour);
    }
    return neighbours;
}

} // namespace fetcher
} // namespace se

#endif // SE_FETCHER_IMPL_HPP
