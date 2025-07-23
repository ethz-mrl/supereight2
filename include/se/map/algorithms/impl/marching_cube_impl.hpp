/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MARCHING_CUBE_IMPL_HPP
#define SE_MARCHING_CUBE_IMPL_HPP

namespace se {

namespace meshing {

/** Functors to select which isosurface to mesh using marching cubes. They return an index to
 * triTable given the data of the 8 neighbouring vertices. See also here:
 * https://en.wikipedia.org/wiki/Marching_cubes#Algorithm
 */
namespace isosurface {

/** Mesh the surface between occupied and free space. This is what's typically wanted when meshing.
 * The mesh faces are oriented away from occupied space.
 */
static constexpr auto occupied = [](const auto data[8]) -> std::uint8_t {
    // A face should be meshed if there are no unknown vertices (not valid) and if there is at least
    // 1 occupied vertex (valid and inside).
    std::uint8_t edge_index = 0;
    for (int i = 0; i < 8; i++) {
        if (!is_valid(data[i])) {
            return 0;
        }
        if (is_inside(data[i])) {
            edge_index |= 1 << i;
        }
    }
    return edge_index;
};

/** Mesh the surface between free and occupied or unknown space. This will create a mesh that
 * encloses all free space in the map. The mesh faces are oriented away from free space.
 */
static constexpr auto free = [](const auto data[8]) -> std::uint8_t {
    // A face should be meshed if there is at least 1 free vertex (valid and not inside). Since both
    // the boundaries between free and unknown, and free and occupied are needed there's no early
    // exit on invalid data.
    std::uint8_t edge_index = 0;
    for (int i = 0; i < 8; i++) {
        if (is_valid(data[i]) && !is_inside(data[i])) {
            edge_index |= 1 << i;
        }
    }
    return edge_index;
};

/** Mesh the surface between free and unknown space, also known as frontiers. The mesh faces are
 * oriented towards unknown space.
 */
static constexpr auto frontier = [](const auto data[8]) -> std::uint8_t {
    // Frontiers are the boundaries between free and unknown space. This means there should be at
    // least 1 free vertex (valid and not inside) and at least 1 unknown vertex (not valid) for a
    // face to be meshed.
    std::uint8_t edge_index = 0;
    int num_valid = 0;
    for (int i = 0; i < 8; i++) {
        if (is_valid(data[i])) {
            num_valid++;
            if (!is_inside(data[i])) {
                edge_index |= 1 << i;
            }
        }
    }
    return num_valid == 8 ? 0 : edge_index;
};

} // namespace isosurface



/// Single-res marching cube implementation

template<typename OctreeT>
Eigen::Vector3f compute_intersection(const OctreeT& octree,
                                     const Eigen::Vector3i& coord_0,
                                     const Eigen::Vector3i& coord_1)
{
    const field_t value_0 = get_field(visitor::getData(octree, coord_0));
    const field_t value_1 = get_field(visitor::getData(octree, coord_1));
    const Eigen::Vector3f point_0_M = coord_0.cast<float>() + se::sample_offset_frac;
    const Eigen::Vector3f point_1_M = coord_1.cast<float>() + se::sample_offset_frac;
    return point_0_M
        + (OctreeT::DataType::surface_boundary - value_0) / (value_1 - value_0)
        * (point_1_M - point_0_M);
}

inline std::pair<Eigen::Vector3i, Eigen::Vector3i>
edge_to_corners(const int edge, const int x, const int y, const int z)
{
    switch (edge) {
    case 0:
        return {Eigen::Vector3i(x, y, z), Eigen::Vector3i(x + 1, y, z)};
    case 1:
        return {Eigen::Vector3i(x + 1, y, z), Eigen::Vector3i(x + 1, y, z + 1)};
    case 2:
        return {Eigen::Vector3i(x + 1, y, z + 1), Eigen::Vector3i(x, y, z + 1)};
    case 3:
        return {Eigen::Vector3i(x, y, z), Eigen::Vector3i(x, y, z + 1)};
    case 4:
        return {Eigen::Vector3i(x, y + 1, z), Eigen::Vector3i(x + 1, y + 1, z)};
    case 5:
        return {Eigen::Vector3i(x + 1, y + 1, z), Eigen::Vector3i(x + 1, y + 1, z + 1)};
    case 6:
        return {Eigen::Vector3i(x + 1, y + 1, z + 1), Eigen::Vector3i(x, y + 1, z + 1)};
    case 7:
        return {Eigen::Vector3i(x, y + 1, z), Eigen::Vector3i(x, y + 1, z + 1)};
    case 8:
        return {Eigen::Vector3i(x, y, z), Eigen::Vector3i(x, y + 1, z)};
    case 9:
        return {Eigen::Vector3i(x + 1, y, z), Eigen::Vector3i(x + 1, y + 1, z)};
    case 10:
        return {Eigen::Vector3i(x + 1, y, z + 1), Eigen::Vector3i(x + 1, y + 1, z + 1)};
    case 11:
        return {Eigen::Vector3i(x, y, z + 1), Eigen::Vector3i(x, y + 1, z + 1)};
    default:
        assert(false && "Invalid edge index");
        return {Eigen::Vector3i::Zero(), Eigen::Vector3i::Zero()};
    }
}

template<typename OctreeT>
Eigen::Vector3f
interp_vertexes(const OctreeT& octree, const int x, const int y, const int z, const int edge)
{
    if (edge < 0 || edge > 11) {
        return Eigen::Vector3f::Zero();
    }

    const auto corners = edge_to_corners(edge, x, y, z);

    return compute_intersection(octree, corners.first, corners.second);
}

template<typename BlockT>
void gather_data(const BlockT* block_ptr,
                 typename BlockT::DataType data_arr[8],
                 const int x,
                 const int y,
                 const int z)
{
    data_arr[0] = block_ptr->data(Eigen::Vector3i(x, y, z));
    data_arr[1] = block_ptr->data(Eigen::Vector3i(x + 1, y, z));
    data_arr[2] = block_ptr->data(Eigen::Vector3i(x + 1, y, z + 1));
    data_arr[3] = block_ptr->data(Eigen::Vector3i(x, y, z + 1));
    data_arr[4] = block_ptr->data(Eigen::Vector3i(x, y + 1, z));
    data_arr[5] = block_ptr->data(Eigen::Vector3i(x + 1, y + 1, z));
    data_arr[6] = block_ptr->data(Eigen::Vector3i(x + 1, y + 1, z + 1));
    data_arr[7] = block_ptr->data(Eigen::Vector3i(x, y + 1, z + 1));
}



template<typename OctreeT>
void gather_data(const OctreeT& octree,
                 typename OctreeT::DataType data[8],
                 const int x,
                 const int y,
                 const int z)
{
    data[0] = se::visitor::getData(octree, Eigen::Vector3i(x, y, z));
    data[1] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y, z));
    data[2] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y, z + 1));
    data[3] = se::visitor::getData(octree, Eigen::Vector3i(x, y, z + 1));
    data[4] = se::visitor::getData(octree, Eigen::Vector3i(x, y + 1, z));
    data[5] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y + 1, z));
    data[6] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y + 1, z + 1));
    data[7] = se::visitor::getData(octree, Eigen::Vector3i(x, y + 1, z + 1));
}



template<typename OctreeT, typename DataToIndexF>
uint8_t compute_index(const OctreeT& octree,
                      const typename OctreeT::BlockType* block_ptr,
                      const int x,
                      const int y,
                      const int z,
                      DataToIndexF data_to_index)
{
    unsigned int block_size = block_ptr->size;
    unsigned int local = ((x % block_size == block_size - 1) << 2)
        | ((y % block_size == block_size - 1) << 1) | ((z % block_size) == block_size - 1);

    typename OctreeT::DataType data[8];
    if (!local) {
        gather_data(block_ptr, data, x, y, z);
    }
    else {
        gather_data(octree, data, x, y, z);
    }

    return data_to_index(data);
}



/// Multi-res marching cube implementation

template<typename DataT>
Eigen::Vector3f compute_dual_intersection(const DataT& data_0,
                                          const DataT& data_1,
                                          const Eigen::Vector3f& dual_point_0_M,
                                          const Eigen::Vector3f& dual_point_1_M)
{
    const field_t value_0 = get_field(data_0);
    const field_t value_1 = get_field(data_1);
    return dual_point_0_M
        + (DataT::surface_boundary - value_0) / (value_1 - value_0)
        * (dual_point_1_M - dual_point_0_M);
}

inline std::pair<int, int> edge_to_corner_indices(const int edge)
{
    switch (edge) {
    case 0:
        return {0, 1};
    case 1:
        return {1, 2};
    case 2:
        return {2, 3};
    case 3:
        return {0, 3};
    case 4:
        return {4, 5};
    case 5:
        return {5, 6};
    case 6:
        return {6, 7};
    case 7:
        return {4, 7};
    case 8:
        return {0, 4};
    case 9:
        return {1, 5};
    case 10:
        return {2, 6};
    case 11:
        return {3, 7};
    default:
        assert(false && "Invalid edge index");
        return {0, 0};
    }
}


template<typename DataT>
Eigen::Vector3f interp_dual_vertexes(const int edge,
                                     const DataT data[8],
                                     const std::array<Eigen::Vector3f, 8>& dual_corner_coords_f)
{
    const auto indices = edge_to_corner_indices(edge);
    return compute_dual_intersection(data[indices.first],
                                     data[indices.second],
                                     dual_corner_coords_f[indices.first],
                                     dual_corner_coords_f[indices.second]);
}



template<typename BlockT, typename DataT>
void gather_dual_data(const BlockT* block_ptr,
                      const int scale,
                      const Eigen::Vector3i& primal_corner_coord,
                      DataT data_arr[8],
                      std::array<Eigen::Vector3f, 8>& dual_corner_coords_f,
                      std::array<Eigen::Vector3i, 8>& dual_corner_coords_i)
{
    const Eigen::Vector3f primal_corner_coord_f = primal_corner_coord.cast<float>();

    // In the local case:        actual_dual_offset = actual_dual_scaling * norm_dual_offset_f and
    // dual_corner_coords_f = primal_corner_coord_f + actual_dual_scaling * norm_dual_offset_f
    const float actual_dual_scaling = (float) octantops::scale_to_size(scale) / 2;
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        dual_corner_coords_i[corner_idx] = primal_corner_coord + logical_dual_offset[corner_idx];
        dual_corner_coords_f[corner_idx] =
            primal_corner_coord_f + actual_dual_scaling * norm_dual_offset_f[corner_idx];
        data_arr[corner_idx] = block_ptr->data(dual_corner_coords_f[corner_idx].cast<int>(), scale);
    }
}

/*! \brief The following strategy is derived from I. Wald, A Simple, General,
 *  and GPU Friendly Method for Computing Dual Mesh and Iso-Surfaces of Adaptive Mesh Refinement (AMR) Data, 2020
 *
 * We validate the scale of all neighbouring blocks need to access the 8 dual corners for each primal corner
 * For each we compute the dual coordinates for primal coordinates with a relative block offset x,y,z in [0, block_size]
 * Due to the fact that block_size is still contained in the offset (rather than [0, block_size - 1], each primal corner
 * is contained in 1 (inside block), 2 (face), 4 (edge) or 8 (corner) neighbouring blocks. We prioritse the block neighbours
 * based on their value (heigher value = higher priority), where the value is calculated via
 * v = 4 << (x is +1) + 2 << (y is +1) + 1 << (z is +1).
 * The minium value is computed for all neighbours for dual corners (c0-8). This means if multiple dual corners fall
 * inside the same neighbouring block we take the minimum value of all the dual corners inside the block.
 * The threshold for populate the lower or higher priority list is the lowest dual corner cost of the corners falling
 * inside the block the relative primal coordinate belongs to.
 *
 * \param[in] primal_corner_coord_rel     relative voxel offset of the primal corner from the block coordinates
 * \param[in] block_size                  size of a voxel block in voxel units
 * \param[in] lower_priority_neighbours   blocks with lower priority, i.e. will be neglected if scale >= scale neighbour
 * \param[in] higher_priority_neighbours  blocks with higher priority, i.e. will only be neglected if scale > scale neighbour
 * \param[in] neighbours                  vector containing a vector with all corner offsets for a neighbouring block.
 *                                        First index is the main block.
 */
inline void norm_dual_corner_idxs(const Eigen::Vector3i& primal_corner_coord_rel,
                                  const int block_size,
                                  BoundedVector<int, 8>& lower_priority_neighbours,
                                  BoundedVector<int, 8>& higher_priority_neighbours,
                                  BoundedVector<BoundedVector<int, 8>, 8>& neighbours)
{
    // 26 binary cases (6 faces, 8 corners, 12 edges)
    // 100 000 upper x crossing
    // 010 000 upper y crossing
    // 001 000 upper z crossing

    // 000 100 lower x crossing
    // 000 010 lower y crossing
    // 000 001 lower z crossing

    unsigned int crossmask = ((primal_corner_coord_rel.x() == block_size) << 5)
        | ((primal_corner_coord_rel.y() == block_size) << 4)
        | ((primal_corner_coord_rel.z() == block_size) << 3)
        | ((primal_corner_coord_rel.x() == 0) << 2) | ((primal_corner_coord_rel.y() == 0) << 1)
        | (primal_corner_coord_rel.z() == 0);

    switch (crossmask) {
    // 6 Faces
    case 1: /* CASE 1 = crosses lower z */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1} and (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1} and (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{2, 3, 6, 7}, {0, 1, 4, 5}};
    } break;
    case 2: /* CASE 2 = crosses lower y */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{4, 5, 6, 7}, {0, 1, 2, 3}};
    } break;
    case 4: /* CASE 3 = crosses lower x */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{1, 2, 5, 6}, {0, 3, 4, 7}};
    } break;
    case 8: /* CASE 4 = crosses upper z */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1} and (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1} and (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {3};
        neighbours = {{0, 1, 4, 5}, {2, 3, 6, 7}};
    } break;
    case 16: /* CASE 5 = crosses upper y */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {4};
        neighbours = {{0, 1, 2, 3}, {4, 5, 6, 7}};
    } break;
    case 32: /* CASE 6 = crosses upper x */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1};
        neighbours = {{0, 3, 4, 7}, {1, 2, 5, 6}};
    } break;


        // 8 Corners
    case 7: /* CASE 7 = crosses lower x(4), y(2), z(1) */
    {
        // Inside
        // (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1},
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 2, 3, 4, 5, 7};
        higher_priority_neighbours = {};
        neighbours = {{6}, {0}, {1}, {2}, {3}, {4}, {5}, {7}};
    } break;
    case 14: /* CASE 8 = crosses lower x(4), y(2) and upper z(8) */
    {
        // Inside
        // (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1},
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1},
        // Higher priority
        // (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 1, 2, 3, 4, 7};
        higher_priority_neighbours = {6};
        neighbours = {{5}, {0}, {1}, {2}, {3}, {4}, {6}, {7}};
    } break;
    case 21: /* CASE 9 = crosses lower x(4), upper y(16) and lower z(1) */
    {
        // Inside
        // (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1},
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1},
        // Higher priority
        // (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 1, 3, 4, 7};
        higher_priority_neighbours = {5, 6};
        neighbours = {{2}, {0}, {1}, {3}, {4}, {5}, {6}, {7}};
    } break;
    case 28: /* CASE 10 = crosses lower x(4) and upper y(16), z(8) */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1},
        // (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3, 4, 7};
        higher_priority_neighbours = {2, 5, 6};
        neighbours = {{1}, {0}, {2}, {3}, {4}, {5}, {6}, {7}};
    } break;
    case 35: /* CASE 11 = crosses upper x(32) and lower y(2), z(1) */
    {
        // Inside
        // (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3, 4};
        higher_priority_neighbours = {1, 2, 5, 6};
        neighbours = {{7}, {0}, {1}, {2}, {3}, {4}, {5}, {6}};
    } break;
    case 42: /* CASE 12 = crosses upper x(32), lower y(2) and upper z(8) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}
        // Higher priority
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1},
        // (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {1, 2, 5, 6, 7};
        neighbours = {{4}, {0}, {1}, {2}, {3}, {5}, {6}, {7}};
    } break;
    case 49: /* CASE 13 = crosses upper x(32), y(16) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}
        // Higher priority
        // (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1},
        // (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 2, 4, 5, 6, 7};
        neighbours = {{3}, {0}, {1}, {2}, {4}, {5}, {6}, {7}};
    } break;
    case 56: /* CASE 14 = crosses upper x(32), y(16), z(8) */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 2, 3, 4, 5, 6, 7};
        neighbours = {{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}};
    } break;


        // 12 Edges
    case 3: /* CASE 15 = crosses lower y(2), z(1) */
    {
        // Inside
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 3, 4};
        higher_priority_neighbours = {};
        neighbours = {{6, 7}, {0, 1}, {2, 3}, {4, 5}};
    } break;
    case 5: /* CASE 16 = crosses lower x(4), z(1) */
    {
        // Inside
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 3};
        higher_priority_neighbours = {};
        neighbours = {{2, 6}, {0, 4}, {3, 7}, {1, 5}};
    } break;
    case 6: /* CASE 17 = crosses lower x(4), y(2) */
    {
        // Inside
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 4};
        higher_priority_neighbours = {};
        neighbours = {{5, 6}, {0, 3}, {4, 7}, {1, 2}};
    } break;
    case 10: /* CASE 18 = crosses lower y(2) and upper z(8) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        // Higher priority
        //
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {7};
        neighbours = {{4, 5}, {0, 1}, {2, 3}, {6, 7}};
    } break;
    case 12: /* CASE 19 = crosses lower x(4) and upper z(8) */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {2};
        neighbours = {{1, 5}, {0, 4}, {3, 7}, {2, 6}};
    } break;
    case 17: /* CASE 20 = crosses upper y(16) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {4, 7};
        neighbours = {{2, 3}, {0, 1}, {4, 5}, {6, 7}};
    } break;
    case 20: /* CASE 21 = crosses lower x(4) and upper y(16)*/
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 4};
        higher_priority_neighbours = {5};
        neighbours = {{1, 2}, {0, 3}, {4, 7}, {5, 6}};
    } break;
    case 24: /* CASE 22 = crosses upper y(16), z(8)*/
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        //
        lower_priority_neighbours = {};
        higher_priority_neighbours = {3, 4, 7};
        neighbours = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};
    } break;
    case 33: /* CASE 23 = crosses upper x(32) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 2};
        neighbours = {{3, 7}, {0, 4}, {1, 5}, {2, 6}};
    } break;
    case 34: /* CASE 24 = crosses upper x(32) and lower y(2) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 5};
        neighbours = {{4, 7}, {0, 3}, {1, 2}, {5, 6}};
    } break;
    case 40: /* CASE 25 = crosses upper x(32), z(8) */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 2, 3};
        neighbours = {{0, 4}, {3, 7}, {1, 5}, {2, 6}};
    } break;
    case 48: /* CASE 26 = crosses upper x(32), y(16) */ {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 4, 5};
        neighbours = {{0, 3}, {4, 7}, {1, 2}, {5, 6}};
    }
    }
}



template<typename OctreeT, typename DataT>
void gather_dual_data(const OctreeT& octree,
                      const typename OctreeT::BlockType* block_ptr,
                      const int scale,
                      const Eigen::Vector3i& primal_corner_coord,
                      DataT data_arr[8],
                      std::array<Eigen::Vector3f, 8>& dual_corner_coords_f,
                      std::array<Eigen::Vector3i, 8>& dual_corner_coords_i)
{
    const int block_scale = block_ptr->current_scale;
    const Eigen::Vector3i primal_corner_coord_rel = primal_corner_coord - block_ptr->coord;

    BoundedVector<int, 8> lower_priority_neighbours, higher_priority_neighbours;
    BoundedVector<BoundedVector<int, 8>, 8> neighbours;
    norm_dual_corner_idxs(primal_corner_coord_rel,
                          block_ptr->size,
                          lower_priority_neighbours,
                          higher_priority_neighbours,
                          neighbours);

    for (const auto& offset_idx : lower_priority_neighbours) {
        Eigen::Vector3i logical_dual_corner_coord =
            primal_corner_coord + logical_dual_offset[offset_idx];
        if (!octree.contains(logical_dual_corner_coord)) {
            set_invalid(data_arr[0]);
            return;
        }
        const auto* const block_neighbour_ptr = static_cast<const typename OctreeT::BlockType*>(
            se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        if (block_neighbour_ptr == nullptr || block_neighbour_ptr->current_scale <= block_scale) {
            set_invalid(data_arr[0]);
            return;
        }
    }
    for (const auto& offset_idx : higher_priority_neighbours) {
        Eigen::Vector3i logical_dual_corner_coord =
            primal_corner_coord + logical_dual_offset[offset_idx];
        if (!octree.contains(logical_dual_corner_coord)) {
            set_invalid(data_arr[0]);
            return;
        }
        const auto* const block_neighbour_ptr = static_cast<const typename OctreeT::BlockType*>(
            se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        if (block_neighbour_ptr == nullptr || block_neighbour_ptr->current_scale < block_scale) {
            set_invalid(data_arr[0]);
            return;
        }
    }

    const int stride = octantops::scale_to_size(scale);
    for (const auto& offset_idx : neighbours[0]) {
        dual_corner_coords_i[offset_idx] = primal_corner_coord + logical_dual_offset[offset_idx];
        dual_corner_coords_f[offset_idx] =
            ((dual_corner_coords_i[offset_idx] / stride) * stride).cast<float>()
            + stride * se::sample_offset_frac; // TODO:  OctreeT<FieldType>::sample_offset_frac_
        data_arr[offset_idx] = block_ptr->data(dual_corner_coords_f[offset_idx].cast<int>(), scale);
    }
    for (size_t neighbour_idx = 1; neighbour_idx < neighbours.size(); ++neighbour_idx) {
        Eigen::Vector3i logical_dual_corner_coord =
            primal_corner_coord + logical_dual_offset[neighbours[neighbour_idx][0]];
        const auto* const block_neighbour_ptr = static_cast<const typename OctreeT::BlockType*>(
            se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        const int neighbour_scale = std::max(block_neighbour_ptr->current_scale, scale);
        const int neighbour_stride = octantops::scale_to_size(neighbour_scale);
        for (const auto& offset_idx : neighbours[neighbour_idx]) {
            dual_corner_coords_i[offset_idx] =
                primal_corner_coord + logical_dual_offset[offset_idx];
            dual_corner_coords_f[offset_idx] =
                ((dual_corner_coords_i[offset_idx] / neighbour_stride) * neighbour_stride)
                    .cast<float>()
                + neighbour_stride
                    * se::sample_offset_frac; // TODO: OctreeT<FieldType>::sample_offset_frac_
            data_arr[offset_idx] = block_neighbour_ptr->data(
                dual_corner_coords_f[offset_idx].cast<int>(), neighbour_scale);
        }
    }
}



template<typename OctreeT, typename DataT, typename DataToIndexF>
void compute_dual_index(const OctreeT& octree,
                        const typename OctreeT::BlockType* block_ptr,
                        const int scale,
                        const Eigen::Vector3i& primal_corner_coord,
                        uint8_t& edge_pattern_idx,
                        DataT data[8],
                        std::array<Eigen::Vector3f, 8>& dual_corner_coords_f,
                        std::array<Eigen::Vector3i, 8>& dual_corner_coords_i,
                        DataToIndexF data_to_index)
{
    const unsigned int block_size = block_ptr->size;
    // The local case is independent of the scale.
    // lower or upper x boundary (block_coord.x() +0 or +block size) -> (binary) 100 -> local += 4
    // lower or upper y boundary (block_coord.y() +0 or +block size) -> (binary) 010 -> local += 2
    // lower or upper z boundary (block_coord.z() +0 or +block size) -> (binary) 001 -> local += 1
    // local = 0 -> local     - dual contains only local primal neightbours
    // local = 1 -> not local - dual crosses z         boundary
    // local = 2 -> not local - dual crosses y         boundary
    // local = 3 -> not local - dual crosses y + z     boundary
    // local = 4 -> not local - dual crosses x         boundary
    // local = 5 -> not local - dual crosses x + z     boundary
    // local = 6 -> not local - dual crosses x + y     boundary
    // local = 7 -> not local - dual crosses x + y + z boundary
    const unsigned int local = ((primal_corner_coord.x() % block_size == 0) << 2)
        | ((primal_corner_coord.y() % block_size == 0) << 1)
        | (primal_corner_coord.z() % block_size == 0);

    if (!local) {
        gather_dual_data(block_ptr,
                         scale,
                         primal_corner_coord,
                         data,
                         dual_corner_coords_f,
                         dual_corner_coords_i);
    }
    else {
        gather_dual_data(octree,
                         block_ptr,
                         scale,
                         primal_corner_coord,
                         data,
                         dual_corner_coords_f,
                         dual_corner_coords_i);
    }

    edge_pattern_idx = data_to_index(data);
}



inline bool checkVertex(const Eigen::Vector3f& vertex_M, const float dim)
{
    return (vertex_M.x() <= 0 || vertex_M.y() <= 0 || vertex_M.z() <= 0 || vertex_M.x() > dim
            || vertex_M.y() > dim || vertex_M.z() > dim);
}



} // namespace meshing



namespace algorithms {



template<typename OctreeT, typename>
typename OctreeT::SurfaceMesh
marching_cube_kernel(const OctreeT& octree,
                     const std::vector<const typename OctreeT::BlockType*>& block_ptrs)
{
    typedef typename OctreeT::SurfaceMesh::value_type Face;

    typename OctreeT::SurfaceMesh mesh;
#pragma omp parallel for
    for (size_t block_idx = 0; block_idx < block_ptrs.size(); block_idx++) {
        const typename OctreeT::BlockType* const block_ptr = block_ptrs[block_idx];

        const Eigen::Vector3i& start_coord = block_ptr->coord;
        const Eigen::Vector3i last_coord =
            (start_coord + Eigen::Vector3i::Constant(OctreeT::BlockType::size))
                .cwiseMin(Eigen::Vector3i::Constant(octree.getSize() - 1));
        for (int x = start_coord.x(); x < last_coord.x(); x++) {
            for (int y = start_coord.y(); y < last_coord.y(); y++) {
                for (int z = start_coord.z(); z < last_coord.z(); z++) {
                    const uint8_t edge_pattern_idx = meshing::compute_index(
                        octree, block_ptr, x, y, z, meshing::isosurface::occupied);
                    const int* const edges = triTable[edge_pattern_idx];
                    for (int e = 0; edges[e] != -1; e += 3) {
                        Face face;
                        for (size_t v = 0; v < Face::num_vertexes; v++) {
                            face.vertexes[v] =
                                meshing::interp_vertexes(octree, x, y, z, edges[e + v]);
                            if (meshing::checkVertex(face.vertexes[v], octree.getSize())) {
                                goto skip_face;
                            }
                        }
                        if constexpr (Face::col_ == Colour::On) {
                            // Using a separate loop than the vertex computation to avoid the
                            // expensive colour interpolation if one of the vertices is invalid and
                            // the whole face is skipped.
                            face.colour.vertexes.emplace();
                            for (size_t v = 0; v < Face::num_vertexes; v++) {
                                const auto colour = visitor::interpColour(octree, face.vertexes[v]);
                                if (!colour) {
                                    face.colour.vertexes = std::nullopt;
                                    break;
                                }
                                (*face.colour.vertexes)[v] = *colour;
                            }
                        }
                        if constexpr (Face::id_ == Id::On) {
                            face.id.id =
                                visitor::getData(octree, block_ptr, Eigen::Vector3i(x, y, z)).id.id;
                        }
#pragma omp critical(marching_cubes)
                        {
                            mesh.push_back(face);
                        }
                    skip_face:;
                    }
                }
            }
        }
    }
    mesh.shrink_to_fit();
    return mesh;
}



template<typename OctreeT, typename>
typename OctreeT::SurfaceMesh
dual_marching_cube_kernel(const OctreeT& octree,
                          const std::vector<const typename OctreeT::BlockType*>& block_ptrs,
                          const int min_desired_scale)
{
    assert(min_desired_scale >= 0);
    assert(min_desired_scale <= OctreeT::BlockType::max_scale);
    typedef typename OctreeT::SurfaceMesh::value_type Face;

    typename OctreeT::SurfaceMesh mesh;
#pragma omp parallel for
    for (size_t block_idx = 0; block_idx < block_ptrs.size(); block_idx++) {
        const typename OctreeT::BlockType* const block_ptr = block_ptrs[block_idx];
        const int voxel_scale =
            std::clamp(min_desired_scale, block_ptr->current_scale, OctreeT::BlockType::max_scale);
        const int voxel_stride = octantops::scale_to_size(voxel_scale);
        const Eigen::Vector3i& start_coord = block_ptr->coord;
        const Eigen::Vector3i last_coord =
            (start_coord + Eigen::Vector3i::Constant(OctreeT::BlockType::size))
                .cwiseMin(Eigen::Vector3i::Constant(octree.getSize() - 1));
        for (int x = start_coord.x(); x <= last_coord.x(); x += voxel_stride) {
            for (int y = start_coord.y(); y <= last_coord.y(); y += voxel_stride) {
                for (int z = start_coord.z(); z <= last_coord.z(); z += voxel_stride) {
                    const Eigen::Vector3i primal_corner_coord = Eigen::Vector3i(x, y, z);

                    if (x == last_coord.x() || y == last_coord.y() || z == last_coord.z()) {
                        if (!se::fetcher::template block<OctreeT>(primal_corner_coord,
                                                                  octree.getRoot())) {
                            continue;
                        }
                    }

                    uint8_t edge_pattern_idx;
                    typename OctreeT::DataType data[8];
                    std::array<Eigen::Vector3f, 8> dual_corner_coords_f;
                    std::array<Eigen::Vector3i, 8> dual_corner_coords_i;
                    dual_corner_coords_f.fill(Eigen::Vector3f::Zero());
                    meshing::compute_dual_index(octree,
                                                block_ptr,
                                                voxel_scale,
                                                primal_corner_coord,
                                                edge_pattern_idx,
                                                data,
                                                dual_corner_coords_f,
                                                dual_corner_coords_i,
                                                meshing::isosurface::occupied);
                    const int* const edges = triTable[edge_pattern_idx];
                    for (int e = 0; edges[e] != -1; e += 3) {
                        Face face;
                        for (size_t v = 0; v < Face::num_vertexes; v++) {
                            face.vertexes[v] = meshing::interp_dual_vertexes(
                                edges[e + v], data, dual_corner_coords_f);
                            if (meshing::checkVertex(face.vertexes[v], octree.getSize())) {
                                goto skip_face;
                            }
                        }
                        face.scale = voxel_scale;
                        if constexpr (Face::col_ == Colour::On) {
                            // Using a separate loop than the vertex computation to avoid the
                            // expensive colour interpolation if one of the vertices is invalid and
                            // the whole face is skipped.
                            face.colour.vertexes.emplace();
                            for (size_t v = 0; v < Face::num_vertexes; v++) {
                                const auto colour =
                                    visitor::interpColour(octree, face.vertexes[v], voxel_scale);
                                if (!colour) {
                                    face.colour.vertexes = std::nullopt;
                                    break;
                                }
                                (*face.colour.vertexes)[v] = *colour;
                            }
                        }
                        if constexpr (Face::id_ == Id::On) {
                            int _;
                            face.id.id =
                                visitor::getData(
                                    octree, block_ptr, Eigen::Vector3i(x, y, z), voxel_scale, _)
                                    .id.id;
                        }
#pragma omp critical(dual_marching_cubes)
                        {
                            mesh.push_back(face);
                        }
                    skip_face:;
                    }
                }
            }
        }
    }
    mesh.shrink_to_fit();
    return mesh;
}

inline uint64_t unique_edge_id(const Eigen::Vector3i& v0, const Eigen::Vector3i& v1)
{
    if (v0.y() == v1.y() && v0.z() == v1.z()) {
        if (v0.x() < v1.x()) {
            return (static_cast<uint64_t>(v0.x()) << 32) | (static_cast<uint64_t>(v0.y()) << 16)
                | static_cast<uint64_t>(v0.z());
        }
        else {
            return (static_cast<uint64_t>(v1.x()) << 32) | (static_cast<uint64_t>(v1.y()) << 16)
                | static_cast<uint64_t>(v1.z());
        }
    }
    if (v0.x() == v1.x() && v0.z() == v1.z()) {
        if (v0.y() < v1.y()) {
            return (uint64_t(1) << 48) | (static_cast<uint64_t>(v0.x()) << 32)
                | (static_cast<uint64_t>(v0.y()) << 16) | static_cast<uint64_t>(v0.z());
        }
        else {
            return (uint64_t(1) << 48) | (static_cast<uint64_t>(v1.x()) << 32)
                | (static_cast<uint64_t>(v1.y()) << 16) | static_cast<uint64_t>(v1.z());
        }
    }
    if (v0.x() == v1.x() && v0.y() == v1.y()) {
        if (v0.z() < v1.z()) {
            return (uint64_t(2) << 48) | (static_cast<uint64_t>(v0.x()) << 32)
                | (static_cast<uint64_t>(v0.y()) << 16) | static_cast<uint64_t>(v0.z());
        }
        else {
            return (uint64_t(2) << 48) | (static_cast<uint64_t>(v1.x()) << 32)
                | (static_cast<uint64_t>(v1.y()) << 16) | static_cast<uint64_t>(v1.z());
        }
    }
    assert(false && "Invalid edge direction (non-manhattan edge)");
    return 0;
}

template<typename OctreeT>
std::vector<meshing::VertexIndexMesh<3>>
dual_marching_cube_kernel_new(const OctreeT& octree,
                              const std::vector<const typename OctreeT::BlockType*>& block_ptrs)
{
    const int octree_size = octree.getSize();
    std::vector<meshing::VertexIndexMesh<3>> block_meshes;

#pragma omp parallel for
    for (size_t block_idx = 0; block_idx < block_ptrs.size(); block_idx++) {
        const typename OctreeT::BlockType* const block_ptr = block_ptrs[block_idx];
        const int voxel_scale = block_ptr->current_scale;
        const int voxel_stride = octantops::scale_to_size(voxel_scale);
        const Eigen::Vector3i& start_coord = block_ptr->coord;
        const Eigen::Vector3i last_coord =
            (start_coord + Eigen::Vector3i::Constant(OctreeT::BlockType::size))
                .cwiseMin(Eigen::Vector3i::Constant(octree_size - 1));

        Eigen::Vector3f vertex_0, vertex_1, vertex_2;

        meshing::VertexIndexMesh<3> block_mesh;
        std::map<uint64_t, size_t> edge_vertex_map;

        for (int x = start_coord.x(); x <= last_coord.x(); x += voxel_stride) {
            for (int y = start_coord.y(); y <= last_coord.y(); y += voxel_stride) {
                for (int z = start_coord.z(); z <= last_coord.z(); z += voxel_stride) {
                    const Eigen::Vector3i primal_corner_coord = Eigen::Vector3i(x, y, z);

                    if (x == last_coord.x() || y == last_coord.y() || z == last_coord.z()) {
                        if (!se::fetcher::template block<OctreeT>(primal_corner_coord,
                                                                  octree.getRoot())) {
                            continue;
                        }
                    }

                    uint8_t edge_pattern_idx;
                    typename OctreeT::DataType data[8];
                    std::array<Eigen::Vector3f, 8> dual_corner_coords_f;
                    std::array<Eigen::Vector3i, 8> dual_corner_coords_i;
                    dual_corner_coords_f.fill(Eigen::Vector3f::Zero());
                    meshing::compute_dual_index(octree,
                                                block_ptr,
                                                voxel_scale,
                                                primal_corner_coord,
                                                edge_pattern_idx,
                                                data,
                                                dual_corner_coords_f,
                                                dual_corner_coords_i,
                                                meshing::isosurface::occupied);
                    const int* const edges = triTable[edge_pattern_idx];
                    for (int e = 0; edges[e] != -1; e += 3) {
                        const auto corner_indices_0 = meshing::edge_to_corner_indices(edges[e]);
                        const auto edge_id_0 =
                            unique_edge_id(dual_corner_coords_i[corner_indices_0.first],
                                           dual_corner_coords_i[corner_indices_0.second]);
                        auto it_0 = edge_vertex_map.find(edge_id_0);

                        const auto corner_indices_1 = meshing::edge_to_corner_indices(edges[e + 1]);
                        const auto edge_id_1 =
                            unique_edge_id(dual_corner_coords_i[corner_indices_1.first],
                                           dual_corner_coords_i[corner_indices_1.second]);
                        auto it_1 = edge_vertex_map.find(edge_id_1);

                        const auto corner_indices_2 = meshing::edge_to_corner_indices(edges[e + 2]);
                        const auto edge_id_2 =
                            unique_edge_id(dual_corner_coords_i[corner_indices_2.first],
                                           dual_corner_coords_i[corner_indices_2.second]);
                        auto it_2 = edge_vertex_map.find(edge_id_2);


                        if (it_0 == edge_vertex_map.end()) {
                            vertex_0 = meshing::compute_dual_intersection(
                                data[corner_indices_0.first],
                                data[corner_indices_0.second],
                                dual_corner_coords_f[corner_indices_0.first],
                                dual_corner_coords_f[corner_indices_0.second]);
                            if (meshing::checkVertex(vertex_0, octree_size)) {
                                continue;
                            }
                        }
                        if (it_1 == edge_vertex_map.end()) {
                            vertex_1 = meshing::compute_dual_intersection(
                                data[corner_indices_1.first],
                                data[corner_indices_1.second],
                                dual_corner_coords_f[corner_indices_1.first],
                                dual_corner_coords_f[corner_indices_1.second]);
                            if (meshing::checkVertex(vertex_1, octree_size)) {
                                continue;
                            }
                        }
                        if (it_2 == edge_vertex_map.end()) {
                            vertex_2 = meshing::compute_dual_intersection(
                                data[corner_indices_2.first],
                                data[corner_indices_2.second],
                                dual_corner_coords_f[corner_indices_2.first],
                                dual_corner_coords_f[corner_indices_2.second]);
                            if (meshing::checkVertex(vertex_2, octree_size)) {
                                continue;
                            }
                        }


                        if (it_0 == edge_vertex_map.end()) {
                            it_0 = edge_vertex_map.insert(it_0,
                                                          {edge_id_0, block_mesh.vertices.size()});
                            block_mesh.vertices.push_back(vertex_0);
                        }
                        block_mesh.indices.push_back(it_0->second);

                        if (it_1 == edge_vertex_map.end()) {
                            it_1 = edge_vertex_map.insert(it_1,
                                                          {edge_id_1, block_mesh.vertices.size()});
                            block_mesh.vertices.push_back(vertex_1);
                        }
                        block_mesh.indices.push_back(it_1->second);

                        if (it_2 == edge_vertex_map.end()) {
                            it_2 = edge_vertex_map.insert(it_2,
                                                          {edge_id_2, block_mesh.vertices.size()});
                            block_mesh.vertices.push_back(vertex_2);
                        }
                        block_mesh.indices.push_back(it_2->second);
                    }
                }
            }
        }
#pragma omp critical(new_marching_cubes)
        {
            block_meshes.emplace_back(block_mesh);
        }
    }
    return block_meshes;
}



template<typename OctreeT>
meshing::VertexIndexMesh<3>
dual_marching_cube_new(const OctreeT& octree,
                       const std::vector<const typename OctreeT::BlockType*>& block_ptrs)
{
    const auto block_meshes = dual_marching_cube_kernel_new(octree, block_ptrs);

    meshing::VertexIndexMesh<3> mesh;
    for (const auto& block_mesh : block_meshes) {
        mesh.merge(block_mesh);
    }

    return mesh;
}



// min_desired_scale is unused in se::Res::Single, prevent compiler warning with [[maybe_unused]].
template<typename OctreeT>
typename OctreeT::SurfaceMesh marching_cube(const OctreeT& octree,
                                            [[maybe_unused]] const int min_desired_scale)
{
    TICK("marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    TICK("marching-cube-create-block-list")
    std::vector<const BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::BlocksIterator<const OctreeT>(&octree);
         block_ptr_itr != se::BlocksIterator<const OctreeT>();
         ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<const BlockType*>(*block_ptr_itr));
    }
    TOCK("marching-cube-create-block-list")

    typename OctreeT::SurfaceMesh mesh;
    if constexpr (OctreeT::res_ == se::Res::Single) {
        mesh = se::algorithms::marching_cube_kernel(octree, block_ptrs);
    }
    else {
        mesh = se::algorithms::dual_marching_cube_kernel(octree, block_ptrs, min_desired_scale);
    }

    TOCK("marching-cube")
    return mesh;
}



template<typename OctreeT>
meshing::VertexIndexMesh<3> dual_marching_cube_new(const OctreeT& octree)
{
    TICK("dual-marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    std::vector<const BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::BlocksIterator<const OctreeT>(&octree);
         block_ptr_itr != se::BlocksIterator<const OctreeT>();
         ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<const BlockType*>(*block_ptr_itr));
    }

    const meshing::VertexIndexMesh<3> mesh =
        se::algorithms::dual_marching_cube_new(octree, block_ptrs);

    TOCK("dual-marching-cube")
    return mesh;
}

} // namespace algorithms
} // namespace se

#endif // SE_MARCHING_CUBE_IMPL_HPP
