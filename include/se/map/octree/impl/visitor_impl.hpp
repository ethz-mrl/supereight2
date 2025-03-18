/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VISITOR_IMPL_HPP
#define SE_VISITOR_IMPL_HPP

namespace se {
namespace visitor {

namespace detail {

/*
* Interpolation's value gather offsets
*/
static const Eigen::Vector3i interp_offsets[8] =
    {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};



template<typename BlockT, typename DataT>
void gather_local(const BlockT* block_ptr,
                  const Eigen::Vector3i& base_coord,
                  DataT neighbour_data[8])
{
    neighbour_data[0] = block_ptr->data(base_coord + interp_offsets[0]);
    neighbour_data[1] = block_ptr->data(base_coord + interp_offsets[1]);
    neighbour_data[2] = block_ptr->data(base_coord + interp_offsets[2]);
    neighbour_data[3] = block_ptr->data(base_coord + interp_offsets[3]);
    neighbour_data[4] = block_ptr->data(base_coord + interp_offsets[4]);
    neighbour_data[5] = block_ptr->data(base_coord + interp_offsets[5]);
    neighbour_data[6] = block_ptr->data(base_coord + interp_offsets[6]);
    neighbour_data[7] = block_ptr->data(base_coord + interp_offsets[7]);
}


template<typename BlockT, typename DataT>
void gather_4(const BlockT* block_ptr,
              const Eigen::Vector3i& base_coord,
              const unsigned int offsets[4],
              DataT neighbour_data[8])
{
    neighbour_data[offsets[0]] = block_ptr->data(base_coord + interp_offsets[offsets[0]]);
    neighbour_data[offsets[1]] = block_ptr->data(base_coord + interp_offsets[offsets[1]]);
    neighbour_data[offsets[2]] = block_ptr->data(base_coord + interp_offsets[offsets[2]]);
    neighbour_data[offsets[3]] = block_ptr->data(base_coord + interp_offsets[offsets[3]]);
}



template<typename BlockT, typename DataT>
void gather_2(const BlockT* block_ptr,
              const Eigen::Vector3i& base_coord,
              const unsigned int offsets[2],
              DataT neighbour_data[8])
{
    neighbour_data[offsets[0]] = block_ptr->data(base_coord + interp_offsets[offsets[0]]);
    neighbour_data[offsets[1]] = block_ptr->data(base_coord + interp_offsets[offsets[1]]);
}



template<typename OctreeT>
bool get_neighbours(const OctreeT& octree,
                    const Eigen::Vector3i& base_coord,
                    typename OctreeT::DataType neighbour_data[8])
{
    unsigned int stride = 1;

    // Check for bounds
    const Eigen::Array3i upper_bounds(base_coord + Eigen::Vector3i::Constant(stride));
    const Eigen::Array3i lower_bounds(base_coord);
    if ((upper_bounds >= octree.getSize()).any() || (lower_bounds < 0).any()) {
        return false;
    }

    unsigned int block_size = OctreeT::BlockType::size;
    unsigned int crossmask = (((base_coord.x() & (block_size - 1)) == block_size - stride) << 2)
        | (((base_coord.y() & (block_size - 1)) == block_size - stride) << 1)
        | ((base_coord.z() & (block_size - 1)) == block_size - stride);

    switch (crossmask) {
    case 0: /* all local */
    {
        const auto* const block_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_ptr) {
            return false;
        }
        gather_local(block_ptr, base_coord, neighbour_data);
    } break;
    case 1: /* z crosses */
    {
        const unsigned int offs1[4] = {0, 1, 2, 3};
        const unsigned int offs2[4] = {4, 5, 6, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    } break;
    case 2: /* y crosses */
    {
        const unsigned int offs1[4] = {0, 1, 4, 5};
        const unsigned int offs2[4] = {2, 3, 6, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    } break;
    case 3: /* y, z cross */
    {
        const unsigned int offs1[2] = {0, 1};
        const unsigned int offs2[2] = {2, 3};
        const unsigned int offs3[2] = {4, 5};
        const unsigned int offs4[2] = {6, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        const auto* const block_3_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs3[0]], octree.getRoot()));
        if (!block_3_ptr) {
            return false;
        }

        const auto* const block_4_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs4[0]], octree.getRoot()));
        if (!block_4_ptr) {
            return false;
        }

        gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
        gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
        gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
    } break;
    case 4: /* x crosses */
    {
        const unsigned int offs1[4] = {0, 2, 4, 6};
        const unsigned int offs2[4] = {1, 3, 5, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    } break;
    case 5: /* x,z cross */
    {
        const unsigned int offs1[2] = {0, 2};
        const unsigned int offs2[2] = {1, 3};
        const unsigned int offs3[2] = {4, 6};
        const unsigned int offs4[2] = {5, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        const auto* const block_3_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs3[0]], octree.getRoot()));
        if (!block_3_ptr) {
            return false;
        }

        const auto* const block_4_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs4[0]], octree.getRoot()));
        if (!block_4_ptr) {
            return false;
        }

        gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
        gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
        gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
    } break;
    case 6: /* x,y cross */
    {
        const unsigned int offs1[2] = {0, 4};
        const unsigned int offs2[2] = {1, 5};
        const unsigned int offs3[2] = {2, 6};
        const unsigned int offs4[2] = {3, 7};

        const auto* const block_1_ptr = static_cast<const typename OctreeT::BlockType*>(
            fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
        if (!block_1_ptr) {
            return false;
        }

        const auto* const block_2_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs2[0]], octree.getRoot()));
        if (!block_2_ptr) {
            return false;
        }

        const auto* const block_3_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs3[0]], octree.getRoot()));
        if (!block_3_ptr) {
            return false;
        }

        const auto* const block_4_ptr =
            static_cast<const typename OctreeT::BlockType*>(fetcher::template block<OctreeT>(
                base_coord + interp_offsets[offs4[0]], octree.getRoot()));
        if (!block_4_ptr) {
            return false;
        }

        gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
        gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
        gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
        gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
    } break;

    case 7: /* x, y, z cross */
    {
        Eigen::Vector3i voxels_coord[8];
        voxels_coord[0] = base_coord + interp_offsets[0];
        voxels_coord[1] = base_coord + interp_offsets[1];
        voxels_coord[2] = base_coord + interp_offsets[2];
        voxels_coord[3] = base_coord + interp_offsets[3];
        voxels_coord[4] = base_coord + interp_offsets[4];
        voxels_coord[5] = base_coord + interp_offsets[5];
        voxels_coord[6] = base_coord + interp_offsets[6];
        voxels_coord[7] = base_coord + interp_offsets[7];

        for (int i = 0; i < 8; ++i) {
            const auto* const block_ptr = static_cast<const typename OctreeT::BlockType*>(
                fetcher::template block<OctreeT>(voxels_coord[i], octree.getRoot()));

            if (!block_ptr) {
                return false;
            }

            neighbour_data[i] = block_ptr->data(voxels_coord[i]);
        }
    } break;
    }
    return true;
}


/////////////////////////////////
/// Multi-res value gathering ///
/////////////////////////////////

template<typename OctreeT, typename DataT>
void gather_local(const OctantBase* leaf_ptr,
                  const Eigen::Vector3i& base_coord,
                  const int scale,
                  DataT neighbour_data[8])
{
    if (leaf_ptr->is_block) {
        const int stride = octantops::scale_to_size(scale);
        const typename OctreeT::BlockType* block_ptr =
            static_cast<const typename OctreeT::BlockType*>(leaf_ptr);
        neighbour_data[0] = block_ptr->data(base_coord + stride * interp_offsets[0], scale);
        neighbour_data[1] = block_ptr->data(base_coord + stride * interp_offsets[1], scale);
        neighbour_data[2] = block_ptr->data(base_coord + stride * interp_offsets[2], scale);
        neighbour_data[3] = block_ptr->data(base_coord + stride * interp_offsets[3], scale);
        neighbour_data[4] = block_ptr->data(base_coord + stride * interp_offsets[4], scale);
        neighbour_data[5] = block_ptr->data(base_coord + stride * interp_offsets[5], scale);
        neighbour_data[6] = block_ptr->data(base_coord + stride * interp_offsets[6], scale);
        neighbour_data[7] = block_ptr->data(base_coord + stride * interp_offsets[7], scale);
    }
    else {
        const typename OctreeT::NodeType* node_ptr =
            static_cast<const typename OctreeT::NodeType*>(leaf_ptr);
        const typename OctreeT::DataType node_data = node_ptr->data();
        std::fill_n(neighbour_data, 8, node_data);
    }
    return;
}



template<typename OctreeT, typename DataT>
void gather_4(const OctantBase* leaf_ptr,
              const Eigen::Vector3i& base_coord,
              const unsigned int offsets[4],
              const int scale,
              DataT neighbour_data[8])
{
    if (leaf_ptr->is_block) {
        const int stride = octantops::scale_to_size(scale);
        const typename OctreeT::BlockType* block_ptr =
            static_cast<const typename OctreeT::BlockType*>(leaf_ptr);
        neighbour_data[offsets[0]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[0]], scale);
        neighbour_data[offsets[1]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[1]], scale);
        neighbour_data[offsets[2]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[2]], scale);
        neighbour_data[offsets[3]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[3]], scale);
    }
    else {
        const typename OctreeT::DataType node_data =
            static_cast<const typename OctreeT::NodeType*>(leaf_ptr)->data();
        neighbour_data[offsets[0]] = node_data;
        neighbour_data[offsets[1]] = node_data;
        neighbour_data[offsets[2]] = node_data;
        neighbour_data[offsets[3]] = node_data;
    }
    return;
}



template<typename OctreeT, typename DataT>
void gather_2(const OctantBase* leaf_ptr,
              const Eigen::Vector3i& base_coord,
              const unsigned int offsets[2],
              const int scale,
              DataT neighbour_data[8])
{
    if (leaf_ptr->is_block) {
        const int stride = octantops::scale_to_size(scale);
        const typename OctreeT::BlockType* block_ptr =
            static_cast<const typename OctreeT::BlockType*>(leaf_ptr);
        neighbour_data[offsets[0]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[0]], scale);
        neighbour_data[offsets[1]] =
            block_ptr->data(base_coord + stride * interp_offsets[offsets[1]], scale);
    }
    else {
        const typename OctreeT::DataType node_data =
            static_cast<const typename OctreeT::NodeType*>(leaf_ptr)->data();
        neighbour_data[offsets[0]] = node_data;
        neighbour_data[offsets[1]] = node_data;
    }
}



template<typename OctreeT>
bool get_neighbours(const OctreeT& octree,
                    const Eigen::Vector3i& base_coord,
                    const int scale,
                    typename OctreeT::DataType neighbour_data[8])
{
    const int stride = octantops::scale_to_size(scale); // Multi-res

    // Check for bounds
    const Eigen::Array3i upper_bounds(base_coord + Eigen::Vector3i::Constant(stride));
    const Eigen::Array3i lower_bounds(base_coord);
    if ((upper_bounds >= octree.getSize()).any() || (lower_bounds < 0).any()) {
        return false;
    }

    const OctantBase* base_octant_ptr =
        fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
    if (!base_octant_ptr) {
        return false;
    }
    const int base_octant_size = (base_octant_ptr->is_block)
        ? OctreeT::BlockType::size
        : static_cast<const typename OctreeT::NodeType*>(base_octant_ptr)->size;

    int crossmask = (((base_coord.x() & (base_octant_size - 1)) == base_octant_size - stride) << 2)
        | (((base_coord.y() & (base_octant_size - 1)) == base_octant_size - stride) << 1)
        | ((base_coord.z() & (base_octant_size - 1)) == base_octant_size - stride);

    if (crossmask != 0 && !base_octant_ptr->is_block) {
        const int block_size = OctreeT::BlockType::size;
        crossmask = (((base_coord.x() & (block_size - 1)) == block_size - stride) << 2)
            | (((base_coord.y() & (block_size - 1)) == block_size - stride) << 1)
            | ((base_coord.z() & (block_size - 1)) == block_size - stride);
    }

    switch (crossmask) {
    case 0: /* all local */
    {
        const OctantBase* leaf_ptr = fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_ptr
            || (leaf_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_local<OctreeT>(leaf_ptr, base_coord, scale, neighbour_data);
    } break;

    case 1: /* z crosses */
    {
        const unsigned int offs1[4] = {0, 1, 2, 3};
        const unsigned int offs2[4] = {4, 5, 6, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_4<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_4<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
    } break;

    case 2: /* y crosses */
    {
        const unsigned int offs1[4] = {0, 1, 4, 5};
        const unsigned int offs2[4] = {2, 3, 6, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_4<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_4<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
    } break;

    case 3: /* y, z cross */
    {
        const unsigned int offs1[2] = {0, 1};
        const unsigned int offs2[2] = {2, 3};
        const unsigned int offs3[2] = {4, 5};
        const unsigned int offs4[2] = {6, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_3_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs3[0]], octree.getRoot());
        if (!leaf_3_ptr
            || (leaf_3_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_3_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_4_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs4[0]], octree.getRoot());
        if (!leaf_4_ptr
            || (leaf_4_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_4_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_2<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_2<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
        gather_2<OctreeT>(leaf_3_ptr, base_coord, offs3, scale, neighbour_data);
        gather_2<OctreeT>(leaf_4_ptr, base_coord, offs4, scale, neighbour_data);
    } break;
    case 4: /* x crosses */
    {
        const unsigned int offs1[4] = {0, 2, 4, 6};
        const unsigned int offs2[4] = {1, 3, 5, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_4<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_4<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
    } break;

    case 5: /* x,z cross */
    {
        const unsigned int offs1[2] = {0, 2};
        const unsigned int offs2[2] = {1, 3};
        const unsigned int offs3[2] = {4, 6};
        const unsigned int offs4[2] = {5, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_3_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs3[0]], octree.getRoot());
        if (!leaf_3_ptr
            || (leaf_3_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_3_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_4_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs4[0]], octree.getRoot());
        if (!leaf_4_ptr
            || (leaf_4_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_4_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_2<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_2<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
        gather_2<OctreeT>(leaf_3_ptr, base_coord, offs3, scale, neighbour_data);
        gather_2<OctreeT>(leaf_4_ptr, base_coord, offs4, scale, neighbour_data);
    } break;

    case 6: /* x,y cross */
    {
        const unsigned int offs1[2] = {0, 4};
        const unsigned int offs2[2] = {1, 5};
        const unsigned int offs3[2] = {2, 6};
        const unsigned int offs4[2] = {3, 7};

        const OctantBase* const leaf_1_ptr =
            fetcher::template leaf<OctreeT>(base_coord, octree.getRoot());
        if (!leaf_1_ptr
            || (leaf_1_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_1_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_2_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs2[0]], octree.getRoot());
        if (!leaf_2_ptr
            || (leaf_2_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_2_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_3_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs3[0]], octree.getRoot());
        if (!leaf_3_ptr
            || (leaf_3_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_3_ptr)->current_scale
                    > scale)) {
            return false;
        }
        const OctantBase* const leaf_4_ptr = fetcher::template leaf<OctreeT>(
            base_coord + stride * interp_offsets[offs4[0]], octree.getRoot());
        if (!leaf_4_ptr
            || (leaf_4_ptr->is_block
                && static_cast<const typename OctreeT::BlockType*>(leaf_4_ptr)->current_scale
                    > scale)) {
            return false;
        }

        gather_2<OctreeT>(leaf_1_ptr, base_coord, offs1, scale, neighbour_data);
        gather_2<OctreeT>(leaf_2_ptr, base_coord, offs2, scale, neighbour_data);
        gather_2<OctreeT>(leaf_3_ptr, base_coord, offs3, scale, neighbour_data);
        gather_2<OctreeT>(leaf_4_ptr, base_coord, offs4, scale, neighbour_data);
    } break;

    case 7: /* x, y, z cross */
    {
        Eigen::Vector3i voxels_coord[8];
        voxels_coord[0] = base_coord + stride * interp_offsets[0];
        voxels_coord[1] = base_coord + stride * interp_offsets[1];
        voxels_coord[2] = base_coord + stride * interp_offsets[2];
        voxels_coord[3] = base_coord + stride * interp_offsets[3];
        voxels_coord[4] = base_coord + stride * interp_offsets[4];
        voxels_coord[5] = base_coord + stride * interp_offsets[5];
        voxels_coord[6] = base_coord + stride * interp_offsets[6];
        voxels_coord[7] = base_coord + stride * interp_offsets[7];

        for (int i = 0; i < 8; ++i) {
            const OctantBase* const leaf_ptr =
                fetcher::template leaf<OctreeT>(voxels_coord[i], octree.getRoot());
            if (!leaf_ptr
                || (leaf_ptr->is_block
                    && static_cast<const typename OctreeT::BlockType*>(leaf_ptr)->current_scale
                        > scale)) {
                return false;
            }

            if (leaf_ptr->is_block) {
                const typename OctreeT::BlockType* block_ptr =
                    static_cast<const typename OctreeT::BlockType*>(leaf_ptr);
                neighbour_data[i] = block_ptr->data(voxels_coord[i], scale);
            }
            else {
                neighbour_data[i] =
                    static_cast<const typename OctreeT::NodeType*>(leaf_ptr)->data();
            }
        }
    } break;
    }
    return true;
}
} // namespace detail



/// Single/multi-res get data functions

template<typename OctreeT>
typename OctreeT::DataType getData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord)
{
    const OctantBase* octant_ptr = fetcher::template leaf<OctreeT>(voxel_coord, octree.getRoot());

    if (!octant_ptr) // not allocated
    {
        return typename OctreeT::DataType();
    }

    return (octant_ptr->is_block)
        ? static_cast<const typename OctreeT::BlockType*>(octant_ptr)->data(voxel_coord)
        : static_cast<const typename OctreeT::NodeType*>(octant_ptr)->data();
}



template<typename OctreeT, typename BlockT>
typename OctreeT::DataType
getData(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord)
{
    assert(block_ptr);

    const Eigen::Vector3i lower_coord = block_ptr->coord;
    const Eigen::Vector3i upper_coord = lower_coord + Eigen::Vector3i::Constant(BlockT::size - 1);
    const bool is_contained = ((voxel_coord.array() >= lower_coord.array())
                               && (voxel_coord.array() <= upper_coord.array()))
                                  .all();
    if (is_contained) {
        return block_ptr->data(voxel_coord);
    }

    return visitor::getData(octree, voxel_coord);
}



/// Multi-res get data functions

template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType>
getData(const OctreeT& octree,
        const Eigen::Vector3i& voxel_coord,
        const int scale_desired,
        int& scale_returned)
{
    const OctantBase* octant_ptr = fetcher::template leaf<OctreeT>(voxel_coord, octree.getRoot());

    if (!octant_ptr) // not allocated
    {
        return typename OctreeT::DataType();
    }

    if (octant_ptr->is_block) {
        return static_cast<const typename OctreeT::BlockType*>(octant_ptr)
            ->data(voxel_coord, scale_desired, scale_returned);
    }
    else {
        scale_returned =
            scale_desired; // TODO: Verify if it should be the node scale or the desired scale.
        return static_cast<const typename OctreeT::NodeType*>(octant_ptr)->data();
    }
}



template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType>
getData(const OctreeT& octree,
        BlockT* block_ptr,
        const Eigen::Vector3i& voxel_coord,
        const int scale_desired,
        int& scale_returned)
{
    assert(block_ptr);

    const Eigen::Vector3i lower_coord = block_ptr->coord;
    const Eigen::Vector3i upper_coord = lower_coord + Eigen::Vector3i::Constant(BlockT::size - 1);
    const bool is_contained = ((voxel_coord.array() >= lower_coord.array())
                               && (voxel_coord.array() <= upper_coord.array()))
                                  .all();
    if (is_contained) {
        return block_ptr->data(voxel_coord, scale_desired, scale_returned);
    }

    return visitor::getData(octree, voxel_coord, scale_desired, scale_returned);
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::DataType::fld_ == Field::Occupancy, typename OctreeT::DataType>
getMinData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired)
{
    const OctantBase* octant_ptr =
        fetcher::template finest_octant<OctreeT>(voxel_coord, scale_desired, octree.getRoot());
    if (!octant_ptr) {
        return typename OctreeT::DataType();
    }
    if (octant_ptr->is_block) {
        int _;
        return static_cast<const typename OctreeT::BlockType*>(octant_ptr)
            ->minData(voxel_coord, scale_desired, _);
    }
    else {
        return static_cast<const typename OctreeT::NodeType*>(octant_ptr)->min_data;
    }
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::DataType::fld_ == Field::Occupancy, typename OctreeT::DataType>
getMaxData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired)
{
    const OctantBase* octant_ptr =
        fetcher::template finest_octant<OctreeT>(voxel_coord, scale_desired, octree.getRoot());

    if (!octant_ptr) // not allocated
    {
        return typename OctreeT::DataType();
    }

    if (octant_ptr->is_block) {
        int scale_returned;
        return static_cast<const typename OctreeT::BlockType*>(octant_ptr)
            ->maxData(voxel_coord, scale_desired, scale_returned);
    }
    else {
        return static_cast<const typename OctreeT::NodeType*>(octant_ptr)->max_data;
    }
}



/// Single/Multi-res get field functions

template<typename OctreeT>
std::optional<field_t> getField(const OctreeT& octree, const Eigen::Vector3i& voxel_coord)
{
    typename OctreeT::DataType data = getData(octree, voxel_coord);
    if (is_valid(data)) {
        return get_field(data);
    }

    return std::nullopt;
}



template<typename OctreeT, typename BlockT>
std::optional<field_t>
getField(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord)
{
    typename OctreeT::DataType data = getData(octree, block_ptr, voxel_coord);
    if (is_valid(data)) {
        return get_field(data);
    }

    return std::nullopt;
}



/// Multi-res get field functions

template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_t>>
getField(const OctreeT& octree,
         const Eigen::Vector3i& voxel_coord,
         const int scale_desired,
         int& scale_returned)
{
    typename OctreeT::DataType data = getData(octree, voxel_coord, scale_desired, scale_returned);
    if (is_valid(data)) {
        return get_field(data);
    }

    return std::nullopt;
}



template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_t>>
getField(const OctreeT& octree,
         BlockT* block_ptr,
         const Eigen::Vector3i& voxel_coord,
         const int scale_desired,
         int& scale_returned)
{
    typename OctreeT::DataType data =
        getData(octree, block_ptr, voxel_coord, scale_desired, scale_returned);
    if (is_valid(data)) {
        return get_field(data);
    }

    return std::nullopt;
}



template<typename OctreeT, typename ValidF, typename GetF>
typename std::enable_if_t<OctreeT::res_ == Res::Multi,
                          std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree,
          const Eigen::Vector3f& voxel_coord_f,
          ValidF valid,
          GetF get,
          const int desired_scale,
          int* const returned_scale)
{
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;

    // Interpolate in a multi-resolution octree.

    // Get the leaf octant containing the query point.
    const OctantBase* const octant_ptr =
        fetcher::template leaf<OctreeT>(voxel_coord_f.cast<int>(), octree.getRoot());
    if (!octant_ptr) {
        return std::nullopt;
    }

    // XXX: Setting the scale to 0 for Nodes to reuse the Block code is an ugly hack.
    const int init_scale = octant_ptr->is_block
        ? std::max(static_cast<const BlockType*>(octant_ptr)->current_scale, desired_scale)
        : 0;

    for (int scale = init_scale; scale <= BlockType::max_scale; scale++) {
        // Subtract the sample offset to get the coordinates of the voxel nearest to the origin out
        // of the 8 voxels nearest to the query point.
        const int stride = octantops::scale_to_size(scale);
        const Eigen::Vector3f base_coord_f = 1.0f / stride * voxel_coord_f - sample_offset_frac;
        const Eigen::Vector3i base_coord = stride * base_coord_f.template cast<int>();
        if (!octree.aabb().contains(base_coord)) {
            return std::nullopt;
        }

        // Gather the data of the 8 voxels nearest to the query point.
        typename OctreeT::DataType data[8] = {};
        if (!detail::get_neighbours(octree, base_coord, scale, data)) {
            continue;
        }
        for (const auto& d : data) {
            if (!valid(d)) {
                return std::nullopt;
            }
        }

        if (returned_scale) {
            // Return the correct scale in the case of Nodes.
            *returned_scale = octant_ptr->is_block
                ? scale
                : octantops::size_to_scale(static_cast<const NodeType*>(octant_ptr)->size);
        }
        // Perform trilinear interpolation.
        // https://en.wikipedia.org/wiki/Trilinear_interpolation#Method
        const Eigen::Vector3f t = math::fracf(base_coord_f);
        const Eigen::Vector3f tc = Eigen::Vector3f::Ones() - t;
        return ((get(data[0]) * tc.x() + get(data[1]) * t.x()) * tc.y()
                + (get(data[2]) * tc.x() + get(data[3]) * t.x()) * t.y())
            * tc.z()
            + ((get(data[4]) * tc.x() + get(data[5]) * t.x()) * tc.y()
               + (get(data[6]) * tc.x() + get(data[7]) * t.x()) * t.y())
            * t.z();
    }
    return std::nullopt;
}



template<typename OctreeT, typename ValidF, typename GetF>
typename std::enable_if_t<OctreeT::res_ == Res::Single,
                          std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, ValidF valid, GetF get)
{
    // Interpolate in a single-resolution octree.

    // Subtract the sample offset to get the coordinates of the voxel nearest to the origin out of
    // the 8 voxels nearest to the query point.
    const Eigen::Vector3f base_coord_f = voxel_coord_f - sample_offset_frac;
    const Eigen::Vector3i base_coord = base_coord_f.template cast<int>();
    // The following top-down view shows the bottom 4 out of the 8 voxels nearest to voxel_coord_f
    // to clarify why the above computation is made.
    //   ┌───────┬───────┐
    //   │       │       │  Legend
    //   │   .   │   .   │  . voxel sample point
    //   │       │  O    │  O voxel_coord_f
    // 1 ├───────┼───────┤  X base_coord_f
    //   │      X│       │  # base_coord
    //   │   .   │   .   │
    //   │       │       │
    // 0 #───────┴───────┘
    //   0       1
    if (!octree.aabb().contains(base_coord)) {
        return std::nullopt;
    }

    // Gather the data of the 8 voxels nearest to the query point.
    typename OctreeT::DataType data[8] = {};
    if (!detail::get_neighbours(octree, base_coord, data)) {
        return std::nullopt;
    }
    for (const auto& d : data) {
        if (!valid(d)) {
            return std::nullopt;
        }
    }

    // Perform trilinear interpolation.
    // https://en.wikipedia.org/wiki/Trilinear_interpolation#Method
    const Eigen::Vector3f t = math::fracf(base_coord_f);
    const Eigen::Vector3f tc = Eigen::Vector3f::Ones() - t;
    return ((get(data[0]) * tc.x() + get(data[1]) * t.x()) * tc.y()
            + (get(data[2]) * tc.x() + get(data[3]) * t.x()) * t.y())
        * tc.z()
        + ((get(data[4]) * tc.x() + get(data[5]) * t.x()) * tc.y()
           + (get(data[6]) * tc.x() + get(data[7]) * t.x()) * t.y())
        * t.z();
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_t>>
getFieldInterp(const OctreeT& octree,
               const Eigen::Vector3f& voxel_coord_f,
               const int desired_scale,
               int* const returned_scale)
{
    return getInterp(
        octree,
        voxel_coord_f,
        [](const typename OctreeT::DataType& d) { return is_valid(d); },
        [](const typename OctreeT::DataType& d) { return get_field(d); },
        desired_scale,
        returned_scale);
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Single, std::optional<field_t>>
getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f)
{
    return getInterp(
        octree,
        voxel_coord_f,
        [](const typename OctreeT::DataType& d) { return is_valid(d); },
        [](const typename OctreeT::DataType& d) { return get_field(d); });
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi && OctreeT::col_ == Colour::On,
                          std::optional<colour_t>>
getColourInterp(const OctreeT& octree,
                const Eigen::Vector3f& voxel_coord_f,
                const int desired_scale,
                int* const returned_scale)
{
    return getInterp(
        octree,
        voxel_coord_f,
        [](const typename OctreeT::DataType& d) { return is_valid(d) && d.colour.weight > 0; },
        [](const typename OctreeT::DataType& d) { return d.colour.colour; },
        desired_scale,
        returned_scale);
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Single && OctreeT::col_ == Colour::On,
                          std::optional<colour_t>>
getColourInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f)
{
    return getInterp(
        octree,
        voxel_coord_f,
        [](const typename OctreeT::DataType& d) { return is_valid(d) && d.colour.weight > 0; },
        [](const typename OctreeT::DataType& d) { return d.colour.colour; });
}



/// Single-res get field gradient functions

template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Single, std::optional<field_vec_t>>
getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f)
{
    const Eigen::Vector3f scaled_voxel_coord_f = voxel_coord_f - sample_offset_frac;
    Eigen::Vector3f factor = math::fracf(scaled_voxel_coord_f);
    const Eigen::Vector3i base_coord = scaled_voxel_coord_f.template cast<int>();

    Eigen::Vector3i lower_lower_coord =
        (base_coord - Eigen::Vector3i::Constant(1)).cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i lower_upper_coord = base_coord.cwiseMax(Eigen::Vector3i::Constant(0));

    Eigen::Vector3i upper_lower_coord =
        (base_coord + Eigen::Vector3i::Constant(1))
            .cwiseMin(Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));
    Eigen::Vector3i upper_upper_coord =
        (base_coord + Eigen::Vector3i::Constant(2))
            .cwiseMin(Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));

    const auto* const block_ptr = static_cast<const typename OctreeT::BlockType*>(
        fetcher::template block<OctreeT>(base_coord, octree.getRoot()));
    if (!block_ptr) {
        return std::nullopt;
    }

    const Eigen::Vector3i grad_coords[32] = {
        Eigen::Vector3i(
            lower_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Unique

        Eigen::Vector3i(
            lower_upper_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            lower_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            lower_upper_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            lower_upper_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

        Eigen::Vector3i(
            upper_lower_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            upper_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            upper_lower_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_lower_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

        Eigen::Vector3i(
            upper_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
        Eigen::Vector3i(
            upper_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()) //< Unique
    };

    field_t grad_field_values[32];

    for (unsigned int i = 0; i < 32; i++) {
        auto grad_field_value = visitor::getField(octree, block_ptr, grad_coords[i]);
        if (!grad_field_value) {
            return std::nullopt;
        }
        grad_field_values[i] = *grad_field_value;
    }

    const float rev_factor_x = (1 - factor.x());
    const float rev_factor_y = (1 - factor.y());
    const float rev_factor_z = (1 - factor.z());

    field_vec_t gradient = Eigen::Vector3f::Constant(0);

    gradient.x() = (((grad_field_values[19] - grad_field_values[0]) * rev_factor_x
                     + (grad_field_values[28] - grad_field_values[7]) * factor.x())
                        * rev_factor_y
                    + ((grad_field_values[23] - grad_field_values[1]) * rev_factor_x
                       + (grad_field_values[29] - grad_field_values[11]) * factor.x())
                        * factor.y())
            * rev_factor_z
        + (((grad_field_values[20] - grad_field_values[2]) * rev_factor_x
            + (grad_field_values[30] - grad_field_values[8]) * factor.x())
               * rev_factor_y
           + ((grad_field_values[24] - grad_field_values[3]) * rev_factor_x
              + (grad_field_values[31] - grad_field_values[12]) * factor.x())
               * factor.y())
            * factor.z();

    gradient.y() = (((grad_field_values[11] - grad_field_values[4]) * rev_factor_x
                     + (grad_field_values[23] - grad_field_values[16]) * factor.x())
                        * rev_factor_y
                    + ((grad_field_values[14] - grad_field_values[7]) * rev_factor_x
                       + (grad_field_values[26] - grad_field_values[19]) * factor.x())
                        * factor.y())
            * rev_factor_z
        + (((grad_field_values[12] - grad_field_values[5]) * rev_factor_x
            + (grad_field_values[24] - grad_field_values[17]) * factor.x())
               * rev_factor_y
           + ((grad_field_values[15] - grad_field_values[8]) * rev_factor_x
              + (grad_field_values[27] - grad_field_values[20]) * factor.x())
               * factor.y())
            * factor.z();

    gradient.z() = (((grad_field_values[8] - grad_field_values[6]) * rev_factor_x
                     + (grad_field_values[20] - grad_field_values[18]) * factor.x())
                        * rev_factor_y
                    + ((grad_field_values[12] - grad_field_values[10]) * rev_factor_x
                       + (grad_field_values[24] - grad_field_values[22]) * factor.x())
                        * factor.y())
            * rev_factor_z
        + (((grad_field_values[9] - grad_field_values[7]) * rev_factor_x
            + (grad_field_values[21] - grad_field_values[19]) * factor.x())
               * rev_factor_y
           + ((grad_field_values[13] - grad_field_values[11]) * rev_factor_x
              + (grad_field_values[25] - grad_field_values[23]) * factor.x())
               * factor.y())
            * factor.z();

    return 0.5f * gradient;
}



/// Multi-res get field gradient functions

template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_vec_t>>
getFieldGrad(const OctreeT& octree,
             const Eigen::Vector3f& voxel_coord_f,
             const int scale_desired,
             int& scale_returned)
{
    typedef typename OctreeT::BlockType BlockType;
    const OctantBase* const octant = fetcher::template finest_octant<OctreeT>(
        voxel_coord_f.cast<int>(), scale_desired, octree.getRoot());
    if (!octant) {
        // Nothing is allocated here, can't compute a gradient.
        return std::nullopt;
    }
    if (!octant->is_block) {
        // The octree is not allocated down to the Block level.
        if constexpr (OctreeT::DataType::fld_ == Field::Occupancy) {
            // Test the Node data.
            const auto& node = *static_cast<const typename OctreeT::NodeType*>(octant);
            if (is_valid(node.data())) {
                // The Node has valid data which should be free space. This part of the map has
                // uniform occupancy, meaning a gradient of 0. This isn't strictly true near the
                // boundary of the node where there can be small non-zero gradients. It's a rather
                // good and simple approximation though.
                scale_returned = octantops::size_to_scale(node.size);
                return field_vec_t::Zero();
            }
            else {
                // The Node has no valid data (unkown space), can't compute a gradient.
                return std::nullopt;
            }
        }
        else {
            // Node-level data is only available in occupancy maps, can't compute a gradient.
            return std::nullopt;
        }
    }
    const BlockType* block_ptr = static_cast<const BlockType*>(octant);

    int init_scale =
        std::max(scale_desired,
                 block_ptr->current_scale); // Get scale to start gradient computation from

    for (int scale = init_scale; scale <= BlockType::max_scale; scale++) {
        scale_returned = scale; // Update returned scale
        const int stride = octantops::scale_to_size(scale);
        const Eigen::Vector3f scaled_voxel_coord_f =
            1.f / stride * voxel_coord_f - sample_offset_frac;
        const Eigen::Vector3f factor = math::fracf(scaled_voxel_coord_f);
        const Eigen::Vector3i base_coord = stride * scaled_voxel_coord_f.template cast<int>();
        Eigen::Vector3i lower_lower_coord = (base_coord - stride * Eigen::Vector3i::Constant(1))
                                                .cwiseMax(Eigen::Vector3i::Constant(0));
        Eigen::Vector3i lower_upper_coord = base_coord.cwiseMax(Eigen::Vector3i::Constant(0));
        Eigen::Vector3i upper_lower_coord =
            (base_coord + stride * Eigen::Vector3i::Constant(1))
                .cwiseMin(Eigen::Vector3i::Constant(octree.getSize())
                          - Eigen::Vector3i::Constant(1));
        Eigen::Vector3i upper_upper_coord =
            (base_coord + stride * Eigen::Vector3i::Constant(2))
                .cwiseMin(Eigen::Vector3i::Constant(octree.getSize())
                          - Eigen::Vector3i::Constant(1));

        const OctantBase* const octant_ptr =
            fetcher::template finest_octant<OctreeT>(base_coord, scale, octree.getRoot());
        if (!octant_ptr) {
            // If this octant isn't allocated there's still a chance a gradient exists at a coarser
            // scale.
            continue;
        }
        // TODO: Is it expected that a node might be queried while computing the gradient or is
        // there some bug in the algorithm?
        if (!octant_ptr->is_block) {
            const auto& node = *static_cast<const typename OctreeT::NodeType*>(octant_ptr);
            if (node.isLeaf() && is_valid(node.data())) {
                // Attempting to compute the gradient at a node, approximate with 0 as before.
                scale_returned = octantops::size_to_scale(node.size);
                return field_vec_t::Zero();
            }
            else {
                // If this node isn't observed there's still a chance a gradient exists at a coarser
                // scale.
                continue;
            }
        }
        block_ptr = static_cast<const BlockType*>(octant_ptr);

        const Eigen::Vector3i grad_coords[32] = {
            Eigen::Vector3i(
                lower_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Unique

            Eigen::Vector3i(
                lower_upper_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_upper_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_upper_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
            Eigen::Vector3i(lower_upper_coord.x(),
                            lower_upper_coord.y(),
                            lower_upper_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(lower_upper_coord.x(),
                            lower_upper_coord.y(),
                            upper_lower_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(
                lower_upper_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_upper_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
            Eigen::Vector3i(lower_upper_coord.x(),
                            upper_lower_coord.y(),
                            lower_upper_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(lower_upper_coord.x(),
                            upper_lower_coord.y(),
                            upper_lower_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(
                lower_upper_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_upper_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                lower_upper_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

            Eigen::Vector3i(
                upper_lower_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_lower_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_lower_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
            Eigen::Vector3i(upper_lower_coord.x(),
                            lower_upper_coord.y(),
                            lower_upper_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(upper_lower_coord.x(),
                            lower_upper_coord.y(),
                            upper_lower_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(
                upper_lower_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_lower_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
            Eigen::Vector3i(upper_lower_coord.x(),
                            upper_lower_coord.y(),
                            lower_upper_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(upper_lower_coord.x(),
                            upper_lower_coord.y(),
                            upper_lower_coord.z()), //< Non-unique 3x
            Eigen::Vector3i(
                upper_lower_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_lower_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_lower_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

            Eigen::Vector3i(
                upper_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
            Eigen::Vector3i(
                upper_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()) //< Unique
        };

        field_t grad_field_values[32];

        bool is_valid = true;

        for (unsigned int i = 0; i < 32; i++) {
            int returned_scale;
            auto grad_field_value = visitor::getField(octree, block_ptr, grad_coords[i], scale, returned_scale);
            if (!grad_field_value || scale != returned_scale) {
                is_valid = false;
                break;
            }
            grad_field_values[i] = *grad_field_value;
        }

        if (!is_valid) {
            continue;
        }

        const float rev_factor_x = (1 - factor.x());
        const float rev_factor_y = (1 - factor.y());
        const float rev_factor_z = (1 - factor.z());

        Eigen::Vector3f gradient = Eigen::Vector3f::Constant(0);

        gradient.x() = (((grad_field_values[19] - grad_field_values[0]) * rev_factor_x
                         + (grad_field_values[28] - grad_field_values[7]) * factor.x())
                            * rev_factor_y
                        + ((grad_field_values[23] - grad_field_values[1]) * rev_factor_x
                           + (grad_field_values[29] - grad_field_values[11]) * factor.x())
                            * factor.y())
                * rev_factor_z
            + (((grad_field_values[20] - grad_field_values[2]) * rev_factor_x
                + (grad_field_values[30] - grad_field_values[8]) * factor.x())
                   * rev_factor_y
               + ((grad_field_values[24] - grad_field_values[3]) * rev_factor_x
                  + (grad_field_values[31] - grad_field_values[12]) * factor.x())
                   * factor.y())
                * factor.z();

        gradient.y() = (((grad_field_values[11] - grad_field_values[4]) * rev_factor_x
                         + (grad_field_values[23] - grad_field_values[16]) * factor.x())
                            * rev_factor_y
                        + ((grad_field_values[14] - grad_field_values[7]) * rev_factor_x
                           + (grad_field_values[26] - grad_field_values[19]) * factor.x())
                            * factor.y())
                * rev_factor_z
            + (((grad_field_values[12] - grad_field_values[5]) * rev_factor_x
                + (grad_field_values[24] - grad_field_values[17]) * factor.x())
                   * rev_factor_y
               + ((grad_field_values[15] - grad_field_values[8]) * rev_factor_x
                  + (grad_field_values[27] - grad_field_values[20]) * factor.x())
                   * factor.y())
                * factor.z();

        gradient.z() = (((grad_field_values[8] - grad_field_values[6]) * rev_factor_x
                         + (grad_field_values[20] - grad_field_values[18]) * factor.x())
                            * rev_factor_y
                        + ((grad_field_values[12] - grad_field_values[10]) * rev_factor_x
                           + (grad_field_values[24] - grad_field_values[22]) * factor.x())
                            * factor.y())
                * rev_factor_z
            + (((grad_field_values[9] - grad_field_values[7]) * rev_factor_x
                + (grad_field_values[21] - grad_field_values[19]) * factor.x())
                   * rev_factor_y
               + ((grad_field_values[13] - grad_field_values[11]) * rev_factor_x
                  + (grad_field_values[25] - grad_field_values[23]) * factor.x())
                   * factor.y())
                * factor.z();

        return 0.5f * gradient / octantops::scale_to_size(scale_returned);
    }

    return std::nullopt;
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_vec_t>>
getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, int& scale_returned)
{
    return getFieldGrad(octree, voxel_coord_f, 0, scale_returned);
}



template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<field_vec_t>>
getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f)
{
    int scale_dummy;
    return getFieldGrad(octree, voxel_coord_f, 0, scale_dummy);
}



} // namespace visitor
} // namespace se

#endif // SE_VISITOR_IMPL_HPP
