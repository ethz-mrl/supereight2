/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/map/octree/octree.hpp>
#include <se/map/octree/visitor.hpp>

// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale)
{
    Eigen::Vector3i adapted_coord;
    adapted_coord.x() = (coord.x() >> scale) << scale;
    adapted_coord.y() = (coord.y() >> scale) << scale;
    adapted_coord.z() = (coord.z() >> scale) << scale;
    return adapted_coord;
}



TEST(VisitorTSDFSingleRes, Interpolation)
{
    Eigen::Vector3i block_coord;
    Eigen::Vector3i coord_ought;
    Eigen::Vector3i coord_is;

    typedef se::TSDFData DataType;

    se::scale_t max_tree_scale = 5;
    unsigned int octree_size = se::octantops::scale_to_size(max_tree_scale);

    // SCALE 0
    constexpr se::scale_t max_block_scale_3 = 3;
    constexpr size_t block_size_3 = se::octantops::scale_to_size(max_block_scale_3);

    typedef se::Octree<DataType, se::Res::Single, block_size_3> OctreeType0;
    typedef OctreeType0::BlockType BlockType0;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> block_coords = {
        Eigen::Vector3i(0, 0, 0),
        Eigen::Vector3i(block_size_3, 0, 0),
        Eigen::Vector3i(0, block_size_3, 0),
        Eigen::Vector3i(block_size_3, block_size_3, 0),
        Eigen::Vector3i(0, 0, block_size_3),
        Eigen::Vector3i(block_size_3, 0, block_size_3),
        Eigen::Vector3i(0, block_size_3, block_size_3),
        Eigen::Vector3i(block_size_3, block_size_3, block_size_3)};

    std::shared_ptr<OctreeType0> octree_ptr_0 =
        std::shared_ptr<OctreeType0>(new OctreeType0(octree_size));

    BlockType0* block_ptr_0 = nullptr;

    for (size_t i = 0; i < block_coords.size(); i++) {
        const Eigen::Vector3i block_coord = block_coords[i];
        coord_ought = adapt_to_scale(block_coord, octree_ptr_0->max_block_scale);
        se::key_t voxel_key;
        se::keyops::encode_key(block_coord, 0, voxel_key);
        block_ptr_0 = static_cast<BlockType0*>(
            se::allocator::block(voxel_key, *octree_ptr_0, octree_ptr_0->getRoot()));
        coord_is = block_ptr_0->coord;
        EXPECT_EQ(coord_ought, coord_is);
        for (size_t voxel_idx = 0; voxel_idx < block_ptr_0->size_cu; voxel_idx++) {
            BlockType0::DataType data;
            data.field.tsdf = i;
            data.field.weight = 1;
            block_ptr_0->setData(voxel_idx, data);
        }
    }

    auto interp_field_value =
        se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 2.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 2.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 8.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 8.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 2.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 2.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 8.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 8.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3.5, *interp_field_value);

    interp_field_value =
        se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(16.f, 16.f, 16.f));
    EXPECT_FALSE(interp_field_value);
}



TEST(VisitorTSDFMultiRes, Interpolation)
{
    Eigen::Vector3i block_coord;
    Eigen::Vector3i coord_ought;
    Eigen::Vector3i coord_is;

    typedef se::TSDFData DataType;

    se::scale_t max_tree_scale = 5;
    unsigned int octree_size = se::octantops::scale_to_size(max_tree_scale);

    // SCALE 0
    constexpr se::scale_t max_block_scale_3 = 3;
    constexpr size_t block_size_3 = se::octantops::scale_to_size(max_block_scale_3);

    typedef se::Octree<DataType, se::Res::Multi, block_size_3> OctreeType0;
    typedef OctreeType0::BlockType BlockType0;

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> block_coords = {
        Eigen::Vector3i(0, 0, 0),
        Eigen::Vector3i(block_size_3, 0, 0),
        Eigen::Vector3i(0, block_size_3, 0),
        Eigen::Vector3i(block_size_3, block_size_3, 0),
        Eigen::Vector3i(0, 0, block_size_3),
        Eigen::Vector3i(block_size_3, 0, block_size_3),
        Eigen::Vector3i(0, block_size_3, block_size_3),
        Eigen::Vector3i(block_size_3, block_size_3, block_size_3)};

    std::shared_ptr<OctreeType0> octree_ptr_0 =
        std::shared_ptr<OctreeType0>(new OctreeType0(octree_size));

    BlockType0* block_ptr_0 = nullptr;

    for (size_t i = 0; i < block_coords.size(); i++) {
        const Eigen::Vector3i block_coord = block_coords[i];
        coord_ought = adapt_to_scale(block_coord, octree_ptr_0->max_block_scale);
        se::key_t voxel_key;
        se::keyops::encode_key(block_coord, 0, voxel_key);
        block_ptr_0 = static_cast<BlockType0*>(
            se::allocator::block(voxel_key, *octree_ptr_0, octree_ptr_0->getRoot()));
        coord_is = block_ptr_0->coord;
        block_ptr_0->current_scale = 1;
        EXPECT_EQ(coord_ought, coord_is);
        for (size_t voxel_idx = block_ptr_0->size_cu; voxel_idx < block_ptr_0->size_cu + 64;
             voxel_idx++) {
            BlockType0::DataType data;
            data.field.tsdf = i;
            data.field.weight = 1;
            block_ptr_0->setData(voxel_idx, data);
        }
    }

    auto interp_field_value =
        se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 2.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 2.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 8.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 8.f, 2.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 2.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 2.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2.5, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(2.f, 8.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3, *interp_field_value);

    interp_field_value = se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(8.f, 8.f, 8.f));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3.5, *interp_field_value);

    interp_field_value =
        se::visitor::getFieldInterp(*octree_ptr_0, Eigen::Vector3f(16.f, 16.f, 16.f));
    EXPECT_FALSE(interp_field_value);
}
