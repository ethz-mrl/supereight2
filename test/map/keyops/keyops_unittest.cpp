/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Eigen/StdVector>
#include <gtest/gtest.h>
#include <se/common/math_util.hpp>
#include <se/common/scale.hpp>
#include <se/map/octant/octant.hpp>
#include <se/map/utils/key_util.hpp>

TEST(KeyOps, EncodeDecodeCode)
{
    // Verify that the code encoding/decoding works correctly
    se::code_t code;
    Eigen::Vector3i coord_is;

    for (int x = 513; x < 1028; x += 19) {
        for (int y = 254; y < 844; y += 13) {
            for (int z = 0; z < 553; z += 11) {
                Eigen::Vector3i coord_ought(x, y, z);
                se::keyops::encode_code(coord_ought, code);
                se::keyops::decode_code(code, coord_is);
                EXPECT_EQ(coord_ought, coord_is);
            }
        }
    }
}

// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale)
{
    Eigen::Vector3i adapted_coord;
    adapted_coord.x() = (coord.x() >> scale) << scale;
    adapted_coord.y() = (coord.y() >> scale) << scale;
    adapted_coord.z() = (coord.z() >> scale) << scale;
    return adapted_coord;
}

TEST(KeyOps, EncodeDecodeKey)
{
    // Verify that the key encoding/decoding works correctly
    se::key_t key;
    se::scale_t scale_is;
    Eigen::Vector3i coord_is;

    for (int x = 513; x < 1028; x += 11) {
        for (int y = 254; y < 844; y += 19) {
            for (int z = 0; z < 553; z += 23) {
                for (se::scale_t scale_ought = 0; scale_ought < 5; ++scale_ought) {
                    Eigen::Vector3i voxel_coord(x, y, z);
                    se::keyops::encode_key(voxel_coord, scale_ought, key);
                    se::keyops::decode_key(key, coord_is, scale_is);

                    Eigen::Vector3i coord_ought = adapt_to_scale(voxel_coord, scale_ought);
                    EXPECT_EQ(coord_ought, coord_is);
                    EXPECT_EQ(scale_ought, scale_is);
                }
            }
        }
    }

    // Verify that bool return correct value if coordinate details is lost
    // when encoding it into a key at a given scale.
    // Return True if no detail is lost, False otherwise.
    Eigen::Vector3i voxel_coord;
    bool full_detail;
    voxel_coord = Eigen::Vector3i(333, 222, 111);
    full_detail = se::keyops::encode_key(voxel_coord, 4, key);
    EXPECT_FALSE(full_detail);

    voxel_coord = Eigen::Vector3i(332, 220, 108);
    full_detail = se::keyops::encode_key(voxel_coord, 2, key);
    EXPECT_TRUE(full_detail);
}

TEST(KeyOps, CodeToChildIdx)
{
    // Verify that the child idx extration from the code is correct
    Eigen::Vector3i coord = Eigen::Vector3i::Zero();
    for (se::scale_t s = 0; s < 8; ++s) {
        coord += se::octantops::scale_to_size(s)
            * Eigen::Vector3i((s & 1) > 0, (s & 2) > 0, (s & 4) > 0);
    }

    se::code_t code;
    se::keyops::encode_code(coord, code);
    se::idx_t child_idx;

    for (se::scale_t s = 0; s < 8; ++s) {
        child_idx = se::keyops::code_to_child_idx(code, s);
        EXPECT_EQ(s, child_idx);
    }
}

TEST(KeyOps, KeyToCodeScale)
{
    // Verify that the code and scale extraction from the key is correct
    se::key_t key;
    se::code_t code;

    se::scale_t scale_is;
    Eigen::Vector3i coord_is;

    for (int x = 513; x < 1028; x += 13) {
        for (int y = 254; y < 844; y += 7) {
            for (int z = 0; z < 553; z += 29) {
                for (se::scale_t scale_ought = 0; scale_ought < 5; ++scale_ought) {
                    Eigen::Vector3i voxel_coord(x, y, z);
                    se::keyops::encode_key(voxel_coord, scale_ought, key);
                    code = se::keyops::key_to_code(key);
                    scale_is = se::keyops::key_to_scale(key);
                    se::keyops::decode_code(code, coord_is);

                    Eigen::Vector3i coord_ought = adapt_to_scale(voxel_coord, scale_ought);
                    EXPECT_EQ(coord_ought, coord_is);
                    EXPECT_EQ(scale_ought, scale_is);
                }
            }
        }
    }
}

TEST(KeyOps, KeyAtScale)
{
    // Verify that the correct key at a given scale is computed
    Eigen::Vector3i coord(170, 204, 240);
    se::key_t base_key;
    se::keyops::encode_key(coord, 0, base_key);

    se::key_t parent_key;
    se::keyops::parent_key(base_key, parent_key);

    for (se::scale_t s = 1; s < 8; ++s) {
        se::key_t key_at_scale;
        se::keyops::key_at_scale(base_key, s, key_at_scale);

        EXPECT_EQ(parent_key, key_at_scale);

        se::keyops::parent_key(parent_key, parent_key);
    }

    // Verify that the correct bool value is returned
    // Return true if the requested key scale is larger or equal to base key's scale
    // Return false otherwise
    // Verify that the correct key at a given scale is computed
    coord = Eigen::Vector3i(160, 192, 240);
    se::keyops::encode_key(coord, 4, base_key);

    se::keyops::parent_key(base_key, parent_key);

    for (se::scale_t s = 0; s < 8; ++s) {
        se::key_t key_at_scale;
        EXPECT_EQ(s >= 4, se::keyops::key_at_scale(base_key, s, key_at_scale));
    }
}

TEST(KeyOps, ParentKey)
{
    Eigen::Vector3i coord(170, 204, 240);

    for (int s = 0; s < 7; ++s) {
        se::key_t child_key;
        se::keyops::encode_key(coord, s, child_key);

        se::key_t parent_key_is;
        se::keyops::parent_key(child_key, parent_key_is);

        se::key_t parent_key_ought;
        se::keyops::encode_key(coord, s + 1, parent_key_ought);

        EXPECT_EQ(parent_key_ought, parent_key_is);
    }
}

TEST(KeyOps, BlockKey)
{
    se::key_t base_key = 0x1FFFFFFFFFFFFE0;
    std::array<se::key_t, 4> keys_ought = {
        0x1FFFFFFFFFFFFE0, 0x1FFFFFFFFFFFF00, 0x1FFFFFFFFFFF800, 0x1FFFFFFFFFFC000};

    for (se::scale_t s = 0; s <= 3; ++s) {
        se::key_t key_is = se::keyops::block_key(base_key, s);
        EXPECT_EQ(keys_ought[s], key_is);
    }
}

TEST(KeyOps, IsChild)
{
    Eigen::Vector3i coord(170, 204, 240);
    se::key_t base_key;
    se::keyops::encode_key(coord, 0, base_key);

    se::key_t parent_key;
    se::key_t child_key;

    for (se::scale_t p = 1; p < 7; ++p) {
        for (se::scale_t c = 0; c < 7; ++c) {
            se::keyops::key_at_scale(base_key, p, parent_key);
            se::keyops::key_at_scale(base_key, c, child_key);
            EXPECT_EQ(
                c < p,
                se::keyops::is_child(parent_key, child_key)); // true if c < p, false otherwise
        }
    }
}

TEST(KeyOps, IsSibling)
{
    Eigen::Vector3i coord(170, 204, 240);

    for (se::scale_t s = 0; s < 7; ++s) {
        Eigen::Vector3i parent_coord = adapt_to_scale(coord, s + 1);
        se::key_t parent_key;
        se::keyops::encode_key(parent_coord, s + 1, parent_key);

        std::array<se::key_t, 8> child_keys;

        for (se::idx_t child_idx = 0; child_idx < 8; ++child_idx) {
            Eigen::Vector3i child_coord = parent_coord
                + se::octantops::scale_to_size(s)
                    * Eigen::Vector3i(
                        (child_idx & 1) > 0, (child_idx & 2) > 0, (child_idx & 4) > 0);
            se::key_t child_key;
            se::keyops::encode_key(child_coord, s, child_key);
            child_keys[child_idx] = child_key;
        }

        for (se::idx_t sib1 = 0; sib1 < 8; ++sib1) {
            for (se::idx_t sib2 = 0; sib2 < 8; ++sib2) {
                EXPECT_TRUE(se::keyops::is_siblings(child_keys[sib1], child_keys[sib2]));
            }
            EXPECT_FALSE(se::keyops::is_siblings(parent_key, child_keys[sib1]));
        }
    }
}

TEST(KeyOps, UniqueKeys)
{
    std::vector<se::key_t> keys;
    keys.reserve(10);
    const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> coords = {
        {56, 12, 12},
        {56, 12, 12},
        {128, 128, 128},
        {128, 132, 130},
        {128, 132, 130},
        {300, 21, 829},
        {628, 36, 227},
        {436, 18, 436},
        {128, 241, 136},
        {128, 241, 136}};
    const std::vector<se::scale_t> scales = {
        0,
        0, // duplicate
        1,
        0,
        1, // duplicate at different scale
        3,
        4,
        2,
        3,
        3 // duplicate
    };
    for (se::idx_t i = 0; i < coords.size(); ++i) {
        se::key_t key_tmp;
        se::keyops::encode_key(coords[i], scales[i], key_tmp);
        keys.push_back(key_tmp);
    }

    se::keyops::sort_keys(keys);
    std::vector<se::key_t> unique_keys;
    se::keyops::unique_keys(keys, unique_keys);

    EXPECT_EQ(8u, unique_keys.size());
    for (se::idx_t i = 1; i < unique_keys.size(); ++i) {
        EXPECT_TRUE(unique_keys[i - 1] != unique_keys[i]);
    }
}

TEST(KeyOps, UniqueAllocation)
{
    const se::scale_t max_block_scale = 2;

    std::vector<se::key_t> keys;
    keys.reserve(10);
    const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> coords = {
        // 0  1  2  3   4  5  6   7   8
        // 1  2  4  8  16 32 64 128 256
        {0, 0, 0},
        {32, 16, 32},
        {40, 24, 32},
        {128, 128, 128},
        {144, 128, 160},
        {144, 132, 164},
        {512, 512, 1024},
        {512, 544, 1056},
        {512, 545, 1056},
        {512, 544, 1056}};
    const std::vector<se::scale_t> scales = {6, 5, 3, 7, 4, 2, 6, 4, 1, 1};
    for (se::idx_t i = 0; i < coords.size(); ++i) {
        se::key_t key_tmp;
        se::keyops::encode_key(coords[i], scales[i], key_tmp);
        keys.push_back(key_tmp);
    }

    se::keyops::sort_keys(keys);
    std::vector<se::key_t> unique_keys;

    se::keyops::unique_allocation(keys, max_block_scale, unique_keys);

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> coords_ought = {
        {40, 24, 32}, {144, 132, 164}, {512, 545, 1056}, {512, 544, 1056}};

    std::vector<se::scale_t> scales_ought = {3, 2, 1, 1};

    std::vector<se::key_t> keys_ought;
    keys_ought.reserve(4);
    for (se::idx_t i = 0; i < coords_ought.size(); ++i) {
        se::key_t key_ought;
        se::keyops::encode_key(coords_ought[i], scales_ought[i], key_ought);
        keys_ought.push_back(key_ought);
    }

    EXPECT_EQ(keys_ought.size(), unique_keys.size());

    for (se::idx_t i = 1; i < unique_keys.size(); ++i) {
        EXPECT_TRUE(unique_keys[i - 1] != unique_keys[i]);
    }

    bool contains = false;
    for (se::idx_t i = 0; i < keys_ought.size(); ++i) // iterate keys_ought
    {
        for (se::idx_t j = 0; j < keys_ought.size(); ++j) // iterate unique_keys
        {
            if (keys_ought[i] == unique_keys[j]) {
                contains = true;
                break;
            }
        }
        if (!contains) {
            break;
        }
    }
    EXPECT_TRUE(contains); // Ought to contain the finer scaled node of the code duplicate
}

TEST(KeyOps, UniqueAtScale)
{
    std::vector<se::key_t> keys;
    keys.reserve(10);
    const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> coords = {
        // 0  1  2  3   4  5  6   7   8
        // 1  2  4  8  16 32 64 128 256
        {0, 0, 0},
        {32, 16, 32},
        {40, 24, 32},
        {128, 128, 128},
        {144, 128, 160},
        {144, 132, 164},
        {512, 512, 1024},
        {512, 544, 1056},
        {512, 545, 1056},
        {513, 545, 1056}};
    const std::vector<se::scale_t> scales = {6, 5, 3, 6, 4, 2, 6, 4, 1, 1};

    for (se::idx_t i = 0; i < coords.size(); ++i) {
        se::key_t key_tmp;
        se::keyops::encode_key(coords[i], scales[i], key_tmp);
        keys.push_back(key_tmp);
    }

    sort(keys.begin(), keys.end());

    // EVALUATE FOR SCALE 5

    se::scale_t scale_ought = 5;
    std::vector<se::key_t> unique_keys;
    se::keyops::unique_at_scale(keys, scale_ought, unique_keys);

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> coords_ought = {
        {32, 0, 32},
        {128, 128, 160},
        {512, 512, 1024},
    };

    std::vector<se::key_t> keys_ought;
    keys_ought.reserve(3);
    for (se::idx_t i = 0; i < coords_ought.size(); ++i) {
        se::key_t key_ought;
        se::keyops::encode_key(coords_ought[i], scale_ought, key_ought);
        keys_ought.push_back(key_ought);
    }

    EXPECT_EQ(keys_ought.size(), unique_keys.size());

    for (se::idx_t i = 1; i < unique_keys.size(); ++i) {
        EXPECT_TRUE(unique_keys[i - 1] != unique_keys[i]);
    }

    bool contains = false;
    for (se::idx_t i = 0; i < keys_ought.size(); ++i) // iterate keys_ought
    {
        for (se::idx_t j = 0; j < keys_ought.size(); ++j) // iterate unique_keys
        {
            if (keys_ought[i] == unique_keys[j]) {
                contains = true;
                break;
            }
        }
        if (!contains) {
            break;
        }
    }
    EXPECT_TRUE(contains); // Ought to contain the finer scaled node of the code duplicate

    scale_ought = 3;
    unique_keys.clear();
    se::keyops::unique_at_scale(keys, scale_ought, unique_keys);

    coords_ought.clear();
    coords_ought = {{40, 24, 32}, {144, 128, 160}, {512, 544, 1056}};

    keys_ought.clear();
    keys_ought.reserve(3);
    for (se::idx_t i = 0; i < coords_ought.size(); ++i) {
        se::key_t key_ought;
        se::keyops::encode_key(coords_ought[i], scale_ought, key_ought);
        keys_ought.push_back(key_ought);
    }

    EXPECT_EQ(keys_ought.size(), unique_keys.size());

    for (se::idx_t i = 1; i < unique_keys.size(); ++i) {
        EXPECT_TRUE(unique_keys[i - 1] != unique_keys[i]);
    }

    contains = false;
    for (se::idx_t i = 0; i < keys_ought.size(); ++i) // iterate keys_ought
    {
        for (se::idx_t j = 0; j < keys_ought.size(); ++j) // iterate unique_keys
        {
            if (keys_ought[i] == unique_keys[j]) {
                contains = true;
                break;
            }
        }
        if (!contains) {
            break;
        }
    }
}
