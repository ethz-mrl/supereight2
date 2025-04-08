/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_HPP
#define SE_OCTREE_HPP

#include <Eigen/Geometry>
#include <memory>
#include <se/map/algorithms/marching_cube.hpp>
#include <se/map/algorithms/mesh.hpp>
#include <se/map/algorithms/structure_meshing.hpp>
#include <se/map/io/mesh_io.hpp>
#include <se/map/octree/iterator.hpp>
#include <se/map/utils/memory_pool.hpp>

namespace se {

/** The octree data structure containing the map data. It is the memory manager of the map since it
 * is the only entity that is able to allocate and deallocate nodes and blocks. There are
 * specialized functions and classes for accessing (se::visitor) and modifying (se::MapIntegrator)
 * the map data as this isn't done through this class. At the maximum octree depth data is stored in
 * blocks of \p BlockSize<sup>3</sup> voxels.
 *
 * https://en.wikipedia.org/wiki/Octree
 *
 * \tparam DataT     The data type stored in the octree.
 * \tparam ResT      se::Res::Single if data is only stored in the octree leaves or se::Res::Multi
 *                   if data is stored in all octree nodes.
 * \tparam BlockSize The edge length of a voxel block in voxels. It must be a power of two.
 */
template<typename DataT, Res ResT = Res::Single, int BlockSize = 8>
class Octree {
    public:
    typedef std::shared_ptr<Octree<DataT, ResT, BlockSize>> Ptr;
    typedef DataT DataType;
    typedef typename DataT::Config DataConfigType;
    typedef Node<DataT, ResT> NodeType;
    typedef Block<DataT, ResT, BlockSize> BlockType;
    typedef TriangleMesh<DataT::col_, DataT::sem_> SurfaceMesh;
    typedef QuadMesh<Colour::Off, Semantics::Off> StructureMesh;

    /** Initialize an octree with an edge length of at least \p size voxels. The actual edge length
     * in voxels, as returned by se::Octree::getSize(), will be the smallest power of 2 that at
     * least \p size and at least `2 * BlockSize`.
     */
    Octree(const int size);

    /** The copy constructor is explicitly deleted because copying octrees is expensive. */
    Octree(const Octree&) = delete;

    /** The copy assignment operator is explicitly deleted because copying octrees is expensive. */
    Octree& operator=(const Octree&) = delete;

    OctreeIterator<Octree<DataT, ResT, BlockSize>> begin();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> begin() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cbegin() const;
    OctreeIterator<Octree<DataT, ResT, BlockSize>> end();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> end() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cend() const;

    /** Return whether point \p voxel_coord with coordinates in voxels is contained in the octree.
     */
    bool contains(const Eigen::Vector3i& voxel_coord) const;

    /** Return a non-null pointer to the octree's root node. */
    OctantBase* getRoot();

    /** Const version of se::Octree::getRoot().
     *
     * \todo Return `const OctantBase*` once a proper const iterator is implemented.
     */
    OctantBase* getRoot() const;

    /** Return the edge length of the octree volume in voxels. */
    int getSize() const;

    /** Return the maximum octree scale, that is, the scale of the root node. */
    int getMaxScale() const;

    /** Return the depth blocks are allocated at. */
    int getBlockDepth() const;

    /** Return a mesh of the reconstructed surface in the octree frame in units of voxels. Apply a
     * transformation, from the octree frame V to some output frame O, \p T_OV to each mesh vertex.
     * For se::Res::Multi maps, only data at scale \p min_desired_scale or coarser will be used to
     * generate the mesh. This allows generating a coarser mesh which is less demanding in terms of
     * computational time and memory. The value of \p min_desired_scale has no effect on
     * se::Res::Single maps.
     */
    SurfaceMesh mesh(const Eigen::Affine3f& T_OV = Eigen::Affine3f::Identity(),
                     const int min_desired_scale = 0) const;

    /** Save the mesh returned by se::Octree::mesh() in \p filename. The \p T_OV and \p
     * min_desired_scale arguments are passed directly to se::Octree::mesh(). The file format will
     * be selected based on the extension of \p filename, which must be one of those in
     * se::io::mesh_extensions. Return the value returned by se::io::save_mesh().
     */
    int saveMesh(const std::string& filename,
                 const Eigen::Affine3f& T_OV = Eigen::Affine3f::Identity(),
                 const int min_desired_scale = 0) const;

    /** Return a mesh of the octree structure in the octree frame in units of voxels. Apply a
     * transformation, from the octree frame V to some output frame O, \p T_OV to each mesh vertex.
     * The returned mesh will only contain leaf octants if \p only_leaves is true.
     */
    StructureMesh structure(const Eigen::Affine3f& T_OV = Eigen::Affine3f::Identity(),
                            const bool only_leaves = true) const;

    /** Save the mesh returned by se::Octree::structure() in \p filename. The \p T_OV and \p
     * only_leaves arguments are passed directly to se::Octree::structure(). The file format will be
     * selected based on the extension of \p filename, which must be one of those in
     * se::io::mesh_extensions. Return the value returned by se::io::save_mesh().
     */
    int saveStructure(const std::string& filename,
                      const Eigen::Affine3f& T_OV = Eigen::Affine3f::Identity(),
                      const bool only_leaves = true) const;

    /** Allocate a child of a node.
     *
     * \note The returned pointer is of type se::OctantBase as the child might be a node or block.
     *
     * \warning This function might be dangerous when using Multires Occupancy. Use
     * se::Octree::allocateChildren() if unsure.
     *
     * \param[in]  parent_ptr The parent of the octant to be allocated.
     * \param[in]  child_idx  The child index of the octant to be allocated.
     * \param[out] child_ptr  The pointer to the allocated or fetched octant.
     *
     * \return True if the child was allocated, false if it was already allocated.
     */
    bool allocate(NodeType* const parent_ptr, const int child_idx, OctantBase*& child_ptr);

    /** Allocate all the children of \p parent_ptr. */
    void allocateChildren(NodeType* const parent_ptr);

    /** Recursively delete all the children of \p parent_ptr. */
    void deleteChildren(NodeType* const parent_ptr);

    /** Return the axis-aligned bounding box of the octree's allocated leaves. The bounding box is
     * computed using the coordinates of allocated voxels, not using the whole allocated volume.
     * Thus the coordinates of its vertices are in the interval [0, se::Octree::getSize()) and it
     * can be used to safely test if some voxel is contained in it using
     * Eigen::AlignedBox3i::contains().
     */
    const Eigen::AlignedBox3i& aabb() const;

    /** Extend the octree allocated leaf AABB to contain the octant with coordinates in voxels \p
     * voxel_coord and edge length in voxels \p size.
     *
     * \note This is typically only needed to update the AABB with leaf nodes as they can't
     * efficiently be detected during allocation since all nodes are leaves when allocated. This
     * function should be called only for newly allocated leaf nodes from an allocator that
     * allocates free nodes (e.g. se::VolumeCarver).
     */
    void aabbExtend(const Eigen::Vector3i& voxel_coord, const int size);

    static constexpr Field fld_ = DataT::fld_;
    static constexpr Colour col_ = DataT::col_;
    static constexpr Semantics sem_ = DataT::sem_;
    static constexpr Res res_ = ResT;
    /** The edge length of a block in voxels. */
    static constexpr int block_size = BlockSize;
    /** The maximum scale of a block. */
    static constexpr scale_t max_block_scale = octantops::size_to_scale(BlockSize);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    const int size_;
    // Allocates and deallocates memory for nodes and blocks.
    MemoryPool<NodeType, BlockType> memory_pool_;
    OctantBase* const root_ptr_; // The pointer lifetime is managed by memory_pool_.
    Eigen::AlignedBox3i aabb_;
};

} // namespace se

#include "impl/octree_impl.hpp"

#endif // SE_OCTREE_HPP
