/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NULL_MEMORY_POOL_HPP
#define SE_NULL_MEMORY_POOL_HPP

#include <Eigen/Core>
#include <set>

namespace se {

/** An alternate implementation of se::MemoryPool that can help debug certain memory-related bugs.
 * Since se::MemoryPool does its own memory management on top of large heap allocations, it can mask
 * certain use-after-free or out-of-bounds-access bugs and make them undetectable to the address
 * sanitizer (ASAN). If you suspect such issues, use se::NullMemoryPool instead.
 */
template<typename NodeT, typename BlockT>
class NullMemoryPool {
    public:
    ~NullMemoryPool()
    {
        for (NodeT* const p : node_buffer_) {
            delete p;
        }
        for (BlockT* const p : block_buffer_) {
            delete p;
        }
    }

    NodeT* allocateRoot(const int octree_size)
    {
        auto [it, _] = node_buffer_.emplace(new NodeT(octree_size, typename NodeT::DataType()));
        return *it;
    }

    NodeT* allocateNode(NodeT* const parent_ptr,
                        const int child_idx,
                        const typename NodeT::DataType& init_data)
    {
        auto [it, _] = node_buffer_.emplace(new NodeT(parent_ptr, child_idx, init_data));
        return *it;
    }

    BlockT* allocateBlock(NodeT* const parent_ptr,
                          const int child_idx,
                          const typename BlockT::DataType& init_data)
    {
        auto [it, _] = block_buffer_.emplace(new BlockT(parent_ptr, child_idx, init_data));
        return *it;
    }

    void deleteNode(NodeT* const node_ptr)
    {
        delete node_ptr;
        node_buffer_.erase(node_ptr);
    }

    void deleteBlock(BlockT* const block_ptr)
    {
        delete block_ptr;
        block_buffer_.erase(block_ptr);
    }

    private:
    std::set<NodeT*> node_buffer_;
    std::set<BlockT*> block_buffer_;
};

} // namespace se

#endif // SE_NULL_MEMORY_POOL_HPP
