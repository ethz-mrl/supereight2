/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_IMPL_HPP
#define SE_OCTREE_IMPL_HPP

namespace se {

template<typename DataT, Res ResT, int BlockSize>
Octree<DataT, ResT, BlockSize>::Octree(const int size) :
        size_(math::power_two_up(std::max(size, 2 * BlockSize))),
        root_ptr_(memory_pool_.allocateRoot(Eigen::Vector3i::Zero(), size_))
{
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>(this);
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin() const
{
    return cbegin();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cbegin() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>(this);
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end() const
{
    return cend();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cend() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>();
}



template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::contains(const Eigen::Vector3i& voxel_coord) const
{
    return voxel_coord.x() >= 0 && voxel_coord.x() < size_ && voxel_coord.y() >= 0
        && voxel_coord.y() < size_ && voxel_coord.z() >= 0 && voxel_coord.z() < size_;
}



template<typename DataT, Res ResT, int BlockSize>
OctantBase* Octree<DataT, ResT, BlockSize>::getRoot()
{
    return root_ptr_;
}



template<typename DataT, Res ResT, int BlockSize>
OctantBase* Octree<DataT, ResT, BlockSize>::getRoot() const
{
    return root_ptr_;
}



template<typename DataT, Res ResT, int BlockSize>
int Octree<DataT, ResT, BlockSize>::getSize() const
{
    return size_;
}



template<typename DataT, Res ResT, int BlockSize>
int Octree<DataT, ResT, BlockSize>::getMaxScale() const
{
    return octantops::size_to_scale(size_);
}



template<typename DataT, Res ResT, int BlockSize>
int Octree<DataT, ResT, BlockSize>::getBlockDepth() const
{
    // The scale of the root is the same as the depth of the leaves.
    return getMaxScale() - octantops::size_to_scale(BlockSize);
}



template<typename DataT, Res ResT, int BlockSize>
typename Octree<DataT, ResT, BlockSize>::SurfaceMesh
Octree<DataT, ResT, BlockSize>::mesh(const Eigen::Affine3f& T_OV, const int min_desired_scale) const
{
    SurfaceMesh mesh = algorithms::marching_cube(*this, min_desired_scale);
    for (auto& face : mesh) {
        for (size_t v = 0; v < SurfaceMesh::value_type::num_vertexes; v++) {
            face.vertexes[v] = T_OV * face.vertexes[v];
        }
    }
    return mesh;
}



template<typename DataT, Res ResT, int BlockSize>
int Octree<DataT, ResT, BlockSize>::saveMesh(const std::string& filename,
                                             const Eigen::Affine3f& T_OV,
                                             const int min_desired_scale) const
{
    return io::save_mesh(mesh(T_OV, min_desired_scale), filename);
}



template<typename DataT, Res ResT, int BlockSize>
typename Octree<DataT, ResT, BlockSize>::StructureMesh
Octree<DataT, ResT, BlockSize>::structure(const Eigen::Affine3f& T_OV, const bool only_leaves) const
{
    StructureMesh mesh = octree_structure_mesh(*this, only_leaves);
    for (auto& face : mesh) {
        for (size_t v = 0; v < StructureMesh::value_type::num_vertexes; v++) {
            face.vertexes[v] = T_OV * face.vertexes[v];
        }
    }
    return mesh;
}



template<typename DataT, Res ResT, int BlockSize>
int Octree<DataT, ResT, BlockSize>::saveStructure(const std::string& filename,
                                                  const Eigen::Affine3f& T_OV,
                                                  const bool only_leaves) const
{
    return io::save_mesh(structure(T_OV, only_leaves), filename);
}



template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::allocate(NodeType* const parent_ptr,
                                              const int child_idx,
                                              OctantBase*& child_ptr)
{
    assert(parent_ptr);

    child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return false;
    }

    const DataT& init_data = parent_ptr->data();
    if (parent_ptr->size == 2 * BlockSize) {
#pragma omp critical(octree_allocator)
        {
            child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
        }
        aabbExtend(child_ptr->coord, parent_ptr->size / 2);
    }
    else {
#pragma omp critical(octree_allocator)
        {
            child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
        }
    }
    parent_ptr->setChild(child_idx, child_ptr);
    return true;
}



template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::allocateChildren(NodeType* const parent_ptr)
{
    assert(parent_ptr);

    const DataT& init_data = parent_ptr->data();
    const bool children_are_blocks = parent_ptr->size == 2 * BlockSize;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        OctantBase* child_ptr = parent_ptr->getChild(child_idx);
        if (child_ptr) {
            continue;
        }
        if (children_are_blocks) {
#pragma omp critical(octree_allocator)
            {
                child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
            }
        }
        else {
#pragma omp critical(octree_allocator)
            {
                child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
            }
        }
        parent_ptr->setChild(child_idx, child_ptr);
    }
    if (children_are_blocks) {
        // Blocks are always leaves and all of them have been allocated, extend the AABB to contain
        // their parent.
        aabbExtend(parent_ptr->coord, parent_ptr->size);
    }
}



template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::deleteChildren(NodeType* const parent_ptr)
{
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        OctantBase* const child_ptr = parent_ptr->getChild(child_idx);
        if (child_ptr) {
            if (child_ptr->is_block) {
                memory_pool_.deleteBlock(static_cast<BlockType*>(child_ptr));
            }
            else {
                auto* const node_ptr = static_cast<NodeType*>(child_ptr);
                deleteChildren(node_ptr);
                memory_pool_.deleteNode(node_ptr);
            }
            parent_ptr->setChild(child_idx, nullptr);
        }
    }
}



template<typename DataT, Res ResT, int BlockSize>
const Eigen::AlignedBox3i& Octree<DataT, ResT, BlockSize>::aabb() const
{
    return aabb_;
}



template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::aabbExtend(const Eigen::Vector3i& voxel_coord, const int size)
{
    const Eigen::AlignedBox3i octant_aabb(voxel_coord,
                                          voxel_coord + Eigen::Vector3i::Constant(size - 1));
    aabb_.extend(octant_aabb);
}

} // namespace se

#endif // SE_OCTREE_IMPL_HPP
