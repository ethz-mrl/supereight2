/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Core>
#include <boost/pool/object_pool.hpp>

namespace se {

/** Manages memory for se::Octree nodes and blocks in an efficient manner. The destructors of all
 * still allocated nodes and blocks are called on destruction of the se::MemoryPool object. User
 * code shouldn't use se::MemoryPool directly as all usage is internal in se::Octree.
 *
 * \warning Never deallocate nodes/blocks created through se::MemoryPool using \p delete.
 *
 * \note Since se::MemoryPool does its own memory management on top of large heap allocations, it
 * can mask certain use-after-free or out-of-bounds-access bugs and make them undetectable to the
 * address sanitizer (ASAN). If you suspect such issues, use se::NullMemoryPool instead.
 */
template<typename NodeT, typename BlockT>
class MemoryPool {
    public:
    /** Allocate the root node with coordinates in voxels \p coord and edge length in voxels \p
     * octree_size.
     */
    NodeT* allocateRoot(const int octree_size)
    {
        return new (node_buffer_.malloc()) NodeT(octree_size, typename NodeT::DataType());
    }

    /** Return a pointer to a newly allocated node that is the child with index \p child_idx of \p
     * parent_ptr and initialize it with \p init_data.
     */
    NodeT* allocateNode(NodeT* const parent_ptr,
                        const int child_idx,
                        const typename NodeT::DataType& init_data)
    {
        return new (node_buffer_.malloc()) NodeT(parent_ptr, child_idx, init_data);
    }

    /** Return a pointer to a newly allocated block that is the child with index \p child_idx of \p
     * parent_ptr and initialise it with \p init_data.
     */
    BlockT* allocateBlock(NodeT* const parent_ptr,
                          const int child_idx,
                          const typename BlockT::DataType& init_data)
    {
        return new (block_buffer_.malloc()) BlockT(parent_ptr, child_idx, init_data);
    }

    /** Destruct and deallocate the node pointed to by \p node_ptr. */
    void deleteNode(NodeT* const node_ptr)
    {
        node_buffer_.destroy(node_ptr);
    }

    /** Destruct and deallocate the block pointed to by \p block_ptr. */
    void deleteBlock(BlockT* const block_ptr)
    {
        block_buffer_.destroy(block_ptr);
    }

    private:
    boost::object_pool<NodeT> node_buffer_;
    boost::object_pool<BlockT> block_buffer_;
};

} // namespace se

#endif // SE_MEMORY_POOL_HPP
