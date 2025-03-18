/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Eigen/StdVector>
#include <gtest/gtest.h>
#include <se/map/octree/octree.hpp>

template<typename OctreeType>
std::shared_ptr<OctreeType>
create_octree()
{
    const int blocks_per_side = 2;
    const int block_size = OctreeType::BlockType::size;
    const int octree_size = blocks_per_side * block_size;
    auto octree_ptr = std::shared_ptr<OctreeType>(new OctreeType(octree_size));

    std::vector<se::key_t> allocation_list;
    allocation_list.reserve(blocks_per_side * blocks_per_side);
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> block_coords;
    for (unsigned y = 0; y < octree_size; y += block_size) {
        for (unsigned z = 0; z < octree_size; z += block_size) {
            Eigen::Vector3i block_coord = Eigen::Vector3i(block_size, y, z);
            se::key_t key;
            se::keyops::encode_key(block_coord, octree_ptr->max_block_scale, key);
            allocation_list.push_back(key);
            block_coords.push_back(block_coord);
        }
    }

    std::sort(allocation_list.begin(), allocation_list.end());
    se::allocator::blocks(allocation_list, *octree_ptr, octree_ptr->getRoot());

    for (auto block_ptr_itr = se::BlocksIterator<OctreeType>(octree_ptr.get());
         block_ptr_itr != se::BlocksIterator<OctreeType>();
         ++block_ptr_itr) {
        auto block_ptr = static_cast<typename OctreeType::BlockType*>(*block_ptr_itr);
        Eigen::Vector3i block_coord = block_ptr->coord;
        for (unsigned x = 0; x < block_size; x++) {
            for (unsigned y = 0; y < block_size; y++) {
                for (unsigned z = 0; z < block_size; z++) {
                    Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                    typename OctreeType::DataType data;
                    if (x < block_size / 2) {
                        data.field.tsdf = 1;
                        data.field.weight = 1;
                        block_ptr->data(voxel_coord) = data;
                    }
                    else {
                        data.field.tsdf = -1;
                        data.field.weight = 1;
                        block_ptr->data(voxel_coord) = data;
                    }
                }
            }
        }
    }

    return octree_ptr;
}

TEST(MeshingTest, EqualScaleNeighbour)
{
    auto octree_ptr = create_octree<se::Octree<se::TSDFData, se::Res::Single, 8>>();

    const auto mesh = se::algorithms::marching_cube(*octree_ptr);

    ASSERT_GT(mesh.size(), 0);

    std::string filename = "multires-mesh-equal-neighbour-single.vtk";
    std::cout << "Saving triangle mesh to file: " << filename << std::endl;
    Eigen::Isometry3f T_MW = Eigen::Isometry3f::Identity();
    se::io::save_mesh_vtk(mesh, filename.c_str(), T_MW.inverse());
}

template<typename OctreeType>
std::shared_ptr<OctreeType>
create_octree_scale()
{
    const int blocks_per_side = 2;
    const int block_size = OctreeType::BlockType::size;
    const int octree_size = blocks_per_side * block_size;
    auto octree_ptr = std::shared_ptr<OctreeType>(new OctreeType(octree_size));

    std::vector<se::key_t> allocation_list;
    allocation_list.reserve(blocks_per_side * blocks_per_side);
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> block_coords;

    for (unsigned y = 0; y < octree_size; y += block_size) {
        for (unsigned z = 0; z < octree_size; z += block_size) {
            Eigen::Vector3i block_coord = Eigen::Vector3i(block_size, y, z);
            se::key_t key;
            se::keyops::encode_key(block_coord, octree_ptr->max_block_scale, key);
            allocation_list.push_back(key);
            block_coords.push_back(block_coord);
        }
    }

    std::sort(allocation_list.begin(), allocation_list.end());
    se::allocator::blocks(allocation_list, *octree_ptr, octree_ptr->getRoot());

    const int curr_scale = 1;
    const int curr_stride = se::octantops::scale_to_size(curr_scale);

    for (auto block_ptr_itr = se::BlocksIterator<OctreeType>(octree_ptr.get());
         block_ptr_itr != se::BlocksIterator<OctreeType>();
         ++block_ptr_itr) {
        auto block_ptr = static_cast<typename OctreeType::BlockType*>(*block_ptr_itr);
        Eigen::Vector3i block_coord = block_ptr->coord;

        block_ptr->current_scale = curr_scale;
        for (unsigned x = 0; x < block_size; x += curr_stride) {
            for (unsigned y = 0; y < block_size; y += curr_stride) {
                for (unsigned z = 0; z < block_size; z += curr_stride) {
                    Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                    typename OctreeType::DataType data;
                    if (x < block_size / 2) {
                        data.field.tsdf = 1;
                        data.field.weight = 1;
                        block_ptr->data(voxel_coord) = data;
                    }
                    else {
                        data.field.tsdf = -1;
                        data.field.weight = 1;
                        block_ptr->data(voxel_coord) = data;
                    }
                }
            }
        }
    }

    return octree_ptr;
}

TEST(MeshingTest, EqualScaleNeighbour2)
{
    auto octree_ptr = create_octree_scale<se::Octree<se::TSDFData, se::Res::Multi, 8>>();

    const auto mesh = se::algorithms::marching_cube(*octree_ptr);
    auto vertex_index_mesh = se::algorithms::dual_marching_cube_new(*octree_ptr);

    ASSERT_GT(mesh.size(), 0);
    ASSERT_EQ(mesh.size() * 3, vertex_index_mesh.indices.size());

    std::string filename = "multires-mesh-equal-neighbour-multi.vtk";
    std::cout << "Saving triangle mesh to file: " << filename << std::endl;
    Eigen::Isometry3f T_MW = Eigen::Isometry3f::Identity();
    se::io::save_mesh_vtk(mesh, filename.c_str(), T_MW.inverse());
}
