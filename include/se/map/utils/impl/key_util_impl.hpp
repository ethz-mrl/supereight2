/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_KEY_UTIL_IMPL_HPP
#define SE_KEY_UTIL_IMPL_HPP

#if defined(SE_TBB) && SE_TBB
#    include <execution>
#    define SE_PARALLEL_SORT(keys) std::sort(std::execution::par_unseq, keys.begin(), keys.end())
#elif __has_include(<parallel/algorithm>)
#    include <parallel/algorithm>
#    define SE_PARALLEL_SORT(keys) __gnu_parallel::sort(keys.begin(), keys.end())
#else
#    define SE_PARALLEL_SORT(keys) se::keyops::sort_keys<se::Sort::SmallToLarge>(keys);
#endif

namespace se {
namespace keyops {

inline bool is_valid(const key_t key, const scale_t limit)
{
    scale_t scale = key & SCALE_MASK;
    ;
    code_t code = key >> SCALE_OFFSET;
    code_t code_remain = code & ~CODE_MASK[scale];
    return scale <= limit && code_remain == 0;
}



inline bool is_valid(const Eigen::Vector3i& coord)
{
    return (coord.x() >= 0 && coord.x() < 524288 && coord.y() >= 0 && coord.y() < 524288
            && coord.z() >= 0 && coord.z() < 524288); // Verify doesn't surpass max coordinates
}



inline code_t expand(unsigned long long value)
{
    assert(value <= 0x7FFFF);      // Limited by 19 bit digits
    key_t x = value & 0x7FFFF;     // Further details will be lost
    x = (x | x << 32) & 0x1f00000000ffff;
    x = (x | x << 16) & 0x1f0000ff0000ff;
    x = (x | x << 8) & 0x100f00f00f00f00f;
    x = (x | x << 4) & 0x10c30c30c30c30c3;
    x = (x | x << 2) & 0x1249249249249249;

    return x;
}



inline key_t compact(uint64_t value)
{
    assert(value <= 0x49249249249249);      // Limited by 19 bit digits
    key_t x = value & 0x49249249249249;     // Further details will be lost
    x = (x | x >> 2) & 0x10c30c30c30c30c3;
    x = (x | x >> 4) & 0x100f00f00f00f00f;
    x = (x | x >> 8) & 0x1f0000ff0000ff;
    x = (x | x >> 16) & 0x1f00000000ffff;
    x = (x | x >> 32) & 0x1fffff;

    return x;
}



inline bool encode_key(const Eigen::Vector3i& coord, const scale_t scale, key_t& key)
{
    assert(scale <= KEY_SCALE_LIMIT); // Verify scale is within key limits
    assert(is_valid(coord));          // Verify doesn't surpass max coordinates

    code_t code_detailed;
    encode_code(coord, code_detailed);
    code_t code = code_detailed & CODE_MASK[scale];
    key = code << SCALE_OFFSET | scale;

    return (code_detailed == code); // Is details lost due to scale?
}



inline key_t encode_key(const Eigen::Vector3i& coord, const scale_t scale)
{
    assert(scale <= KEY_SCALE_LIMIT); // Verify scale is within key limits
    assert(is_valid(coord));          // Verify doesn't surpass max coordinates

    code_t code_detailed;
    encode_code(coord, code_detailed);
    code_t code = code_detailed & CODE_MASK[scale];

    assert(code_detailed == code); // Is details lost due to scale?

    return code << SCALE_OFFSET | scale;
}



inline bool encode_key(const key_t code, const scale_t scale, key_t& key)
{
    assert(scale <= KEY_SCALE_LIMIT); // Verify scale is within key limits

    code_t code_filtered = code & CODE_MASK[scale];

    key = code << SCALE_OFFSET | scale;

    return (code == code_filtered);
}



inline key_t encode_key(const key_t code, const scale_t scale)
{
    assert(scale <= KEY_SCALE_LIMIT); // Verify scale is within key limits

    // Ensure the code is the same as the filtered code.
    assert(code == (code & CODE_MASK[scale]));

    return code << SCALE_OFFSET | scale;
}



inline void decode_key(const key_t key, Eigen::Vector3i& coord, scale_t& scale)
{
    scale = key_to_scale(key);
    code_t code = key_to_code(key);

    assert(is_valid(key)); // Verify key is valid

    decode_code(code, coord);
}



inline void encode_code(const Eigen::Vector3i& coord, code_t& code)
{
    assert(is_valid(coord)); // Verify doesn't surpass max coordinates

    key_t x = expand(coord.x());
    key_t y = expand(coord.y()) << 1;
    key_t z = expand(coord.z()) << 2;
    code = x | y | z;
}



inline void decode_code(const code_t code, Eigen::Vector3i& coord)
{
    coord = Eigen::Vector3i(compact(code >> 0ull), compact(code >> 1ull), compact(code >> 2ull));
}



inline idx_t code_to_child_idx(const code_t code, const scale_t child_scale)
{
    assert(child_scale <= KEY_SCALE_LIMIT);       // Verify scale is within key limits
    return (code >> (child_scale * NUM_DIM)) & 7; // The 7 filters the last 3 bits 111
}



inline code_t key_to_code(const key_t key)
{
    assert(is_valid(key)); // Verify key is valid

    return key >> SCALE_OFFSET;
}



inline Eigen::Vector3i key_to_coord(const key_t key)
{
    assert(is_valid(key)); // Verify key is valid

    code_t code = key_to_code(key);

    Eigen::Vector3i coord;
    decode_code(code, coord);
    return coord;
}



inline scale_t key_to_scale(const key_t key)
{
    assert(is_valid(key)); // Verify key is valid

    return key & SCALE_MASK;
}



inline bool key_at_scale(const key_t key, const scale_t scale, key_t& key_at_scale)
{
    assert(is_valid(key));

    key_at_scale = (key & (CODE_MASK[scale] << SCALE_OFFSET)) | scale;

    return (key_to_scale(key) <= scale);
}



inline void parent_key(const key_t child_key, key_t& parent_key)
{
    assert(is_valid(child_key, KEY_SCALE_LIMIT - 1)); // Verify key is valid
    const scale_t parent_scale = key_to_scale(child_key) + 1;
    parent_key = (child_key & (CODE_MASK[parent_scale] << SCALE_OFFSET)) | parent_scale;
}



inline key_t block_key(const key_t key, const scale_t max_block_scale)
{
    assert(is_valid(key)); // Verify key is valid
    return (key & (CODE_MASK[max_block_scale] << SCALE_OFFSET | SCALE_MASK));
}



inline void
parent_to_child_key(const key_t parent_key, const code_t code_at_scale, key_t& child_key)
{
    scale_t child_scale = key_to_scale(parent_key) - 1;
    child_key = parent_key | (code_at_scale << ((child_scale * NUM_DIM) + SCALE_OFFSET));
    child_key = (child_key & ~SCALE_MASK) | child_scale;
}



inline bool is_child(const key_t parent_key, const key_t child_key)
{
    assert(is_valid(child_key));                       // Verify child is valid
    assert(is_valid(parent_key, KEY_SCALE_LIMIT - 1)); // Verify parent is valid

    const int parent_scale = key_to_scale(parent_key);
    const int child_scale = key_to_scale(child_key);

    if (child_scale >= parent_scale) {
        return false;
    }

    const code_t parent_code = key_to_code(parent_key);
    const code_t child_code = key_to_code(child_key) & CODE_MASK[parent_scale];

    return (parent_code ^ child_code) == 0;
}



inline bool is_siblings(const key_t sibling_1_key, const key_t sibling_2_key)
{
    assert(is_valid(sibling_1_key)); // Verify sibling 1 is valid
    assert(is_valid(sibling_2_key)); // Verify sibling 2 is valid

    const scale_t sibling_1_scale = key_to_scale(sibling_1_key);
    const scale_t sibling_2_scale = key_to_scale(sibling_2_key);

    if (sibling_1_scale != sibling_2_scale) {
        return false;
    }

    const scale_t parent_scale = sibling_1_scale + 1;
    const code_t sibling_1_code = key_to_code(sibling_1_key) & CODE_MASK[parent_scale];
    const code_t sibling_2_code = key_to_code(sibling_2_key) & CODE_MASK[parent_scale];

    return (sibling_1_code ^ sibling_2_code) == 0;
}



template<>
inline void sort_keys<Sort::SmallToLarge>(std::vector<key_t>& keys)
{
    std::sort(keys.begin(), keys.end(), [](key_t i, key_t j) { return (i < j); });
}



template<>
inline void sort_keys<Sort::LargeToSmall>(std::vector<key_t>& keys)
{
    std::sort(keys.begin(), keys.end(), [](key_t i, key_t j) { return (i > j); });
}



template<Safe SafeB = Safe::On>
inline void unique_keys(std::vector<key_t>& keys, std::vector<key_t>& unique_keys)
{
    if (keys.size() == 0) {
        return;
    }

    if constexpr (SafeB == Safe::On) {
        // Sort keys smallest to largest
        SE_PARALLEL_SORT(keys);
    }

    unique_keys.push_back(keys.front());
    for (auto const& key : keys) {
        if (unique_keys.back() != key) {
            unique_keys.push_back(key);
        }
    }
}



template<Safe SafeB = Safe::On>
inline void unique_allocation(std::vector<key_t>& keys,
                              const scale_t max_block_scale,
                              std::vector<key_t>& unique_keys)
{
    if (keys.size() == 0) {
        return;
    }

    if constexpr (SafeB == Safe::On) {
        // Sort keys smallest to largest
        SE_PARALLEL_SORT(keys);
    }

    const key_t init_key = keyops::block_key(keys.front(), max_block_scale);
    unique_keys.push_back(init_key);

    for (auto const& key : keys) {
        key_t block_key = keyops::block_key(key, max_block_scale);
        if (is_child(unique_keys.back(), block_key)) {
            unique_keys.back() = block_key;
        }
        else if (unique_keys.back() != block_key) {
            unique_keys.push_back(block_key);
        }
    }
}



template<Safe SafeB = Safe::On>
inline void
unique_at_scale(std::vector<key_t>& keys, const scale_t scale, std::vector<key_t>& unique_keys)
{
    if (keys.size() == 0) {
        return;
    }

    if constexpr (SafeB == Safe::On) {
        // Sort keys smallest to largest
        SE_PARALLEL_SORT(keys);
    }

    key_t key_at_scale;
    idx_t restart_idx = 1;
    for (auto key_itr = keys.begin(); key_itr != keys.end(); ++key_itr) {
        if (keyops::key_at_scale(*key_itr, scale, key_at_scale)) {
            unique_keys.push_back(key_at_scale);
            break;
        }
        ++restart_idx;
    }

    for (auto key_itr = keys.begin() + restart_idx; key_itr != keys.end(); ++key_itr) {
        if (keyops::key_at_scale(*key_itr, scale, key_at_scale)
            && unique_keys.back() != key_at_scale) {
            unique_keys.push_back(key_at_scale);
        }
    }
}



} // namespace keyops
} // namespace se

#undef SE_PARALLEL_SORT

#endif // SE_KEY_UTIL_IMPL_HPP
