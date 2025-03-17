/*
 * SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <queue>
#include <se/integrator/map_integrator.hpp>
#include <sstream>

typedef se::OccupancyMap<> MapType;
typedef MapType::OctreeType OctreeType;
typedef MapType::OctreeType::NodeType NodeType;
typedef MapType::OctreeType::BlockType BlockType;
typedef MapType::OctreeType::DataType DataType;



static void expect_valid_block_at_scale(const BlockType& block,
                                        const Eigen::Vector3i& parent_coord,
                                        const int parent_scale)
{
    std::stringstream fail_msg;
    fail_msg << "for block (" << block.coord.transpose() << ") in voxel ("
             << parent_coord.transpose() << ") at scale " << parent_scale;

    const int parent_size = se::octantops::scale_to_size(parent_scale);
    const int child_scale = parent_scale - 1;
    const auto& parent_min_data = block.getMinData(parent_coord, parent_scale);
    const auto& parent_max_data = block.getMaxData(parent_coord, parent_scale);
    // Iterate over the 8 children.
    int observed_children = 0;
    for (int z = 0; z < parent_size; z += parent_size / 2) {
        for (int y = 0; y < parent_size; y += parent_size / 2) {
            for (int x = 0; x < parent_size; x += parent_size / 2) {
                const Eigen::Vector3i child_coord = parent_coord + Eigen::Vector3i(x, y, z);

                const auto& child_min_data = block.getMinData(child_coord, child_scale);
                if (child_min_data.field.observed) {
                    EXPECT_GE(se::get_field(child_min_data), se::get_field(parent_min_data))
                        << fail_msg.str();
                    observed_children++;
                }

                const auto& child_max_data = block.getMaxData(child_coord, child_scale);
                if (child_max_data.field.observed) {
                    EXPECT_LE(se::get_field(child_max_data), se::get_field(parent_max_data))
                        << fail_msg.str();
                }
            }
        }
    }

    EXPECT_EQ(parent_min_data.field.observed, observed_children == 8) << fail_msg.str();
    EXPECT_EQ(parent_max_data.field.observed, observed_children == 8) << fail_msg.str();
}



static void expect_valid_block(const BlockType& block)
{
    std::stringstream fail_msg;
    fail_msg << "for block (" << block.coord.transpose() << ")";

    // The octree root is an se::Node so all blocks must have a parent.
    EXPECT_TRUE(block.parent()) << fail_msg.str();
    EXPECT_TRUE(block.isLeaf()) << fail_msg.str();

    // Test that the data of all children are within the minimum and maximum data of their
    // parent. Skip the finest scale since it has no children.
    for (int scale = block.getMaxScale(); scale > block.getCurrentScale(); scale--) {
        const int size_at_scale = se::octantops::scale_to_size(scale);
        // Iterate over all the octants at this scale.
        for (int z = 0; z < BlockType::size; z += size_at_scale) {
            for (int y = 0; y < BlockType::size; y += size_at_scale) {
                for (int x = 0; x < BlockType::size; x += size_at_scale) {
                    const Eigen::Vector3i coord = block.coord + Eigen::Vector3i(x, y, z);
                    expect_valid_block_at_scale(block, coord, scale);
                }
            }
        }
    }
}



static void expect_valid_leaf_node(const NodeType& node)
{
    std::stringstream fail_msg;
    fail_msg << "for node (" << node.coord.transpose() << ") with size " << node.size;

    EXPECT_TRUE(node.isLeaf()) << fail_msg.str();

    // Leaf nodes with at least one integration (.valid()) should fully reflect their contained
    // volume (.observed).
    if (node.min_data.field.valid()) {
        EXPECT_TRUE(node.min_data.field.observed) << fail_msg.str();
    }
    if (node.max_data.field.valid()) {
        EXPECT_TRUE(node.max_data.field.observed) << fail_msg.str();
    }

    for (int child_idx = 0; child_idx < 8; child_idx++) {
        EXPECT_FALSE(node.getChild(child_idx)) << fail_msg.str();
    }
}



static void expect_valid_non_leaf_node(const NodeType& node)
{
    std::stringstream fail_msg;
    fail_msg << "for node (" << node.coord.transpose() << ") with size " << node.size;

    EXPECT_FALSE(node.isLeaf()) << fail_msg.str();

    const DataType& min_data = node.min_data;
    const DataType& max_data = node.max_data;
    int observed_children = 0;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        const se::OctantBase* const child_ptr = node.getChild(child_idx);
        if (child_ptr) {
            const int child_size = child_ptr->is_block
                ? static_cast<const BlockType*>(child_ptr)->getSize()
                : static_cast<const NodeType*>(child_ptr)->size;
            EXPECT_EQ(child_size, node.size / 2);

            const DataType& child_min_data = child_ptr->is_block
                ? static_cast<const BlockType*>(child_ptr)->getMinData()
                : static_cast<const NodeType*>(child_ptr)->min_data;
            if (child_min_data.field.observed) {
                EXPECT_GE(se::get_field(child_min_data), se::get_field(min_data)) << fail_msg.str();
                observed_children++;
            }

            const DataType& child_max_data = child_ptr->is_block
                ? static_cast<const BlockType*>(child_ptr)->getMaxData()
                : static_cast<const NodeType*>(child_ptr)->max_data;
            if (child_max_data.field.observed) {
                EXPECT_LE(se::get_field(child_max_data), se::get_field(max_data)) << fail_msg.str();
            }
        }
    }

    EXPECT_EQ(min_data.field.observed, observed_children == 8) << fail_msg.str();
    EXPECT_EQ(max_data.field.observed, observed_children == 8) << fail_msg.str();
}



static void expect_valid_node(const NodeType& node)
{
    std::stringstream fail_msg;
    fail_msg << "for node (" << node.coord.transpose() << ") with size " << node.size;

    // Minimum and maximum node data must always be consistent with each other.
    EXPECT_EQ(node.min_data.field.valid(), node.max_data.field.valid()) << fail_msg.str();
    EXPECT_EQ(node.min_data.field.observed, node.max_data.field.observed) << fail_msg.str();

    if (node.isLeaf()) {
        expect_valid_leaf_node(node);
    }
    else {
        expect_valid_non_leaf_node(node);
    }
}



static void expect_valid_octant(const se::OctantBase* const octant_ptr, const int octree_size)
{
    std::stringstream fail_msg;
    fail_msg << "for octant (" << octant_ptr->coord << ") ";

    ASSERT_TRUE(octant_ptr) << fail_msg.str();
    EXPECT_GE(octant_ptr->coord.x(), 0) << fail_msg.str();
    EXPECT_GE(octant_ptr->coord.y(), 0) << fail_msg.str();
    EXPECT_GE(octant_ptr->coord.z(), 0) << fail_msg.str();
    EXPECT_LT(octant_ptr->coord.x(), octree_size) << fail_msg.str();
    EXPECT_LT(octant_ptr->coord.y(), octree_size) << fail_msg.str();
    EXPECT_LT(octant_ptr->coord.z(), octree_size) << fail_msg.str();

    if (octant_ptr->is_block) {
        expect_valid_block(*static_cast<const BlockType*>(octant_ptr));
    }
    else {
        expect_valid_node(*static_cast<const NodeType*>(octant_ptr));
    }
}



static void expect_valid_octree(const OctreeType& octree)
{
    // Traverse the octree breadth-first and test the validity of all octants.
    std::queue<const se::OctantBase*> octant_ptrs;
    octant_ptrs.push(octree.getRoot());
    while (!octant_ptrs.empty()) {
        const se::OctantBase* const octant_ptr = octant_ptrs.front();
        octant_ptrs.pop();
        expect_valid_octant(octant_ptr, octree.getSize());
        // Add the child octants to the queue.
        if (!octant_ptr->is_block) {
            const NodeType& node = *static_cast<const NodeType*>(octant_ptr);
            for (int child_idx = 0; child_idx < 8; child_idx++) {
                const se::OctantBase* const child_ptr = node.getChild(child_idx);
                if (child_ptr) {
                    octant_ptrs.push(child_ptr);
                }
            }
        }
    }
}



TEST(integrateDepth, occupancy)
{
    const se::PinholeCamera sensor = se::PinholeCamera::testInstance();

    MapType map(Eigen::Vector3f::Constant(12.8f), 0.1f);
    expect_valid_octree(map.getOctree());

    se::MapIntegrator integrator(map);
    int frame = 0;

    // Integrate a wall perpendicular to the x axis at 10 m.
    se::Image<float> depth(sensor.model.imageWidth(), sensor.model.imageHeight(), 10.0f);
    Eigen::Isometry3f T_WB(Eigen::Translation3f(Eigen::Vector3f(-5.0f, 0.0f, 0.0f)));
    integrator.integrateDepth(frame++,
                              se::Measurements{se::Measurement{depth, sensor, T_WB * sensor.T_BS}});
    expect_valid_octree(map.getOctree());

    // Move closer and integrate again.
    constexpr float delta = 1.0f; // m
    for (size_t i = 0; i < depth.size(); i++) {
        depth[i] -= delta;
    }
    T_WB.translation().x() += delta;
    integrator.integrateDepth(frame++,
                              se::Measurements{se::Measurement{depth, sensor, T_WB * sensor.T_BS}});
    expect_valid_octree(map.getOctree());
}
