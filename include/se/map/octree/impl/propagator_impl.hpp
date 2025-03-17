/*
 * SPDX-FileCopyrightText: 2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PROPAGATOR_IMPL_HPP
#define SE_PROPAGATOR_IMPL_HPP

namespace se {
namespace propagator {



template<typename OctreeT, typename AggregateF>
void propagateBlockUp(const OctreeT& /* octree */,
                      se::OctantBase* octant_ptr,
                      const int init_scale,
                      AggregateF aggregate_children_funct)
{
    typedef typename OctreeT::BlockType BlockType;
    typedef typename BlockType::DataType DataType;
    typedef typename BlockType::DataUnion DataUnionType;

    assert(octant_ptr);
    assert((octant_ptr->is_block));
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    assert(init_scale >= block_ptr->current_scale);

    const Eigen::Vector3i block_coord = block_ptr->coord;
    const int block_size = BlockType::size;

    for (int child_scale = init_scale; child_scale < octantops::size_to_scale(block_size);
         ++child_scale) {
        const int child_stride = octantops::scale_to_size(child_scale);
        const int parent_stride = child_stride << 1;

        for (int z = 0; z < block_size; z += parent_stride) {
            for (int y = 0; y < block_size; y += parent_stride) {
                for (int x = 0; x < block_size; x += parent_stride) {
                    const Eigen::Vector3i parent_coord = block_coord + Eigen::Vector3i(x, y, z);

                    int child_idx = 0;
                    std::array<DataType, 8> child_data;

                    for (int k = 0; k < parent_stride; k += child_stride) {
                        for (int j = 0; j < parent_stride; j += child_stride) {
                            for (int i = 0; i < parent_stride; i += child_stride) {
                                const Eigen::Vector3i child_coord =
                                    parent_coord + Eigen::Vector3i(i, j, k);
                                child_data[child_idx++] =
                                    block_ptr->getData(child_coord, child_scale);
                            } // i
                        }     // j
                    }         // k

                    DataUnionType parent_data_union =
                        block_ptr->getDataUnion(parent_coord, child_scale + 1);

                    aggregate_children_funct(parent_data_union, child_data);

                    block_ptr->setDataUnion(parent_data_union);
                } // x
            }     // y
        }         // z

    } // scale
}



template<typename OctreeT, typename ChildF, typename ParentF>
void propagateBlockDown(const OctreeT& octree,
                        se::OctantBase* octant_ptr,
                        const int target_scale,
                        ChildF child_funct,
                        ParentF parent_funct)
{
    typedef typename OctreeT::BlockType BlockType;
    typedef typename BlockType::DataUnion DataUnionType;

    assert(octant_ptr);
    assert((octant_ptr->is_block));
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    assert(target_scale <= block_ptr->current_scale);

    const Eigen::Vector3i block_coord = block_ptr->coord;
    const int block_size = BlockType::size;
    for (int parent_scale = block_ptr->current_scale; parent_scale > target_scale; --parent_scale) {
        const int parent_stride = octantops::scale_to_size(parent_scale);
        for (int z = 0; z < block_size; z += parent_stride) {
            for (int y = 0; y < block_size; y += parent_stride) {
                for (int x = 0; x < block_size; x += parent_stride) {
                    const Eigen::Vector3i parent_coord = block_coord + Eigen::Vector3i(x, y, z);
                    DataUnionType parent_data_union =
                        block_ptr->getDataUnion(parent_coord, parent_scale);
                    const int child_stride = parent_stride / 2;
                    const int child_scale = parent_scale - 1;

                    for (int k = 0; k < parent_stride; k += child_stride) {
                        for (int j = 0; j < parent_stride; j += child_stride) {
                            for (int i = 0; i < parent_stride; i += child_stride) {
                                const Eigen::Vector3i child_coord =
                                    parent_coord + Eigen::Vector3i(i, j, k);
                                DataUnionType child_data_union =
                                    block_ptr->getDataUnion(child_coord, child_scale);

                                child_funct(octree, block_ptr, child_data_union, parent_data_union);
                                block_ptr->setDataUnion(child_data_union);
                            } // i
                        }     // j
                    }         // k

                    parent_funct(octree, block_ptr, parent_data_union);
                    block_ptr->setDataUnion(parent_data_union);
                } // x
            }     // y
        }         // z
    }             // scale
}



template<typename PropagateF>
void propagateToRoot(std::vector<std::unordered_set<se::OctantBase*>> octant_ptrs,
                     PropagateF& propagate_funct)
{
    for (int d = octant_ptrs.size() - 1; d > 0; d--) {
        std::unordered_set<se::OctantBase*> child_ptrs_at_depth = octant_ptrs[d];
        std::unordered_set<se::OctantBase*>::iterator child_ptr_itr;

        for (child_ptr_itr = child_ptrs_at_depth.begin();
             child_ptr_itr != child_ptrs_at_depth.end();
             ++child_ptr_itr) {
            se::OctantBase* child_ptr = *child_ptr_itr;
            se::OctantBase* parent_ptr = child_ptr->parent();
            assert(parent_ptr);

            propagate_funct(child_ptr, parent_ptr);

            octant_ptrs[d - 1].insert(parent_ptr);
        }

    } // d
}



template<typename PropagateF>
void propagateToRoot(std::vector<se::OctantBase*>& octant_ptrs, PropagateF& propagate_funct)
{
    TICK("propagate-nodes-vector")

    std::unordered_set<se::OctantBase*> child_ptrs;
    std::unordered_set<se::OctantBase*> parent_ptrs;

    for (const auto& child_ptr : octant_ptrs) {
        se::OctantBase* parent_ptr = child_ptr->parent();
        assert(parent_ptr);

        if (child_ptr->timestamp > parent_ptr->timestamp) {
            parent_ptr->timestamp = child_ptr->timestamp;
            child_ptrs.insert(parent_ptr);
        }
    } // block_ptrs

    while (!parent_ptrs.empty()) {
        std::unordered_set<se::OctantBase*>::iterator child_ptr_itr;

        for (child_ptr_itr = child_ptrs.begin(); child_ptr_itr != child_ptrs.end();
             ++child_ptr_itr) {
            se::OctantBase* child_ptr = *child_ptr_itr;
            se::OctantBase* parent_ptr = child_ptr->parent();

            if (parent_ptr) {
                propagate_funct(child_ptr, parent_ptr);
                parent_ptrs.insert(parent_ptr);
            }
        }

        std::swap(child_ptrs, parent_ptrs);
        parent_ptrs.clear();

    } // while

    TOCK("propagate-nodes-vector")
}



static inline void propagateTimeStampToRoot(std::vector<se::OctantBase*>& octant_ptrs)
{
    auto time_step_prop = [](se::OctantBase* child_ptr, se::OctantBase* parent_ptr) {
        if (child_ptr->timestamp > parent_ptr->timestamp) {
            parent_ptr->timestamp = child_ptr->timestamp;
        }
    };

    propagateToRoot(octant_ptrs, time_step_prop);
}



} // namespace propagator
} // namespace se



#endif // SE_PROPAGATOR_IMPL_HPP
