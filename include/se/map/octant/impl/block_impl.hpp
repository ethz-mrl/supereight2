/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_BLOCK_IMPL_HPP
#define SE_BLOCK_IMPL_HPP

namespace se {

/// Single-res Block ///

template<typename DataT, int BlockSize, typename DerivedT>
BlockSingleRes<DataT, BlockSize, DerivedT>::BlockSingleRes(const DataType init_data)
{
    block_data_.fill(init_data); // TODO: Verify that initialisation doesn't cause regression
}


template<typename DataT, int BlockSize, typename DerivedT>
const typename BlockSingleRes<DataT, BlockSize, DerivedT>::DataType&
BlockSingleRes<DataT, BlockSize, DerivedT>::getData(const int voxel_idx) const
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<typename DataT, int BlockSize, typename DerivedT>
typename BlockSingleRes<DataT, BlockSize, DerivedT>::DataType&
BlockSingleRes<DataT, BlockSize, DerivedT>::getData(const int voxel_idx)
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<typename DataT, int BlockSize, typename DerivedT>
const typename BlockSingleRes<DataT, BlockSize, DerivedT>::DataType&
BlockSingleRes<DataT, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord) const
{
    const Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
    const int voxel_idx = voxel_offset.x() + voxel_offset.y() * underlying()->size
        + voxel_offset.z() * underlying()->size_sq;
    return getData(voxel_idx);
}



template<typename DataT, int BlockSize, typename DerivedT>
typename BlockSingleRes<DataT, BlockSize, DerivedT>::DataType&
BlockSingleRes<DataT, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord)
{
    const Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
    const int voxel_idx = voxel_offset.x() + voxel_offset.y() * underlying()->size
        + voxel_offset.z() * underlying()->size_sq;
    return getData(voxel_idx);
}



template<typename DataT, int BlockSize, typename DerivedT>
void BlockSingleRes<DataT, BlockSize, DerivedT>::setData(const Eigen::Vector3i& voxel_coord,
                                                         const DataType& data)
{
    const Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
    const int voxel_idx = voxel_offset.x() + voxel_offset.y() * underlying()->size
        + voxel_offset.z() * underlying()->size_sq;
    setData(voxel_idx, data);
}



template<typename DataT, int BlockSize, typename DerivedT>
void BlockSingleRes<DataT, BlockSize, DerivedT>::setData(const int voxel_idx, const DataType& data)
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    block_data_[voxel_idx] = data;
}



/// Multi-res Block ///

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes(
    const DataType init_data)
{
    block_data_.fill(init_data); // TODO: Verify that initialisation doesn't cause regression
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
int BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getVoxelIdx(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    assert(scale >= 0);
    assert(scale <= max_scale);
    const Eigen::Vector3i voxel_offset = (voxel_coord - underlying()->coord) / (1 << scale);
    const int size_at_scale = size_at_scales_[scale];
    return scale_offsets_[scale] + voxel_offset.x() + voxel_offset.y() * size_at_scale
        + voxel_offset.z() * math::sq(size_at_scale);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const int voxel_idx) const
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const int voxel_idx)
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord) const
{
    return getData(voxel_coord, current_scale);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord)
{
    return getData(voxel_coord, current_scale);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_desired,
    int& scale_returned) const
{
    scale_returned = std::max(scale_desired, current_scale);
    return getData(getVoxelIdx(voxel_coord, scale_returned));
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_desired,
    int& scale_returned)
{
    scale_returned = std::max(scale_desired, current_scale);
    return getData(getVoxelIdx(voxel_coord, scale_returned));
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    return getData(getVoxelIdx(voxel_coord, scale));
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    return getData(getVoxelIdx(voxel_coord, scale));
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataUnion
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getDataUnion(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    const int voxel_idx = getVoxelIdx(voxel_coord, scale);
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    assert(static_cast<size_t>(voxel_idx) < block_past_data_.size());
    DataUnion data_union;
    data_union.coord = voxel_coord;
    data_union.scale = scale;
    data_union.data = block_data_[voxel_idx];
    data_union.past_data = block_past_data_[voxel_idx];
    data_union.data_idx = voxel_idx;
    data_union.prop_data_idx = voxel_idx;
    return data_union;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataUnion
BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getDataUnion(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    const int voxel_idx = getVoxelIdx(voxel_coord, scale);
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    assert(static_cast<size_t>(voxel_idx) < block_past_data_.size());
    DataUnion data_union;
    data_union.coord = voxel_coord;
    data_union.scale = scale;
    data_union.data = block_data_[voxel_idx];
    data_union.past_data = block_past_data_[voxel_idx];
    data_union.data_idx = voxel_idx;
    data_union.prop_data_idx = voxel_idx;
    return data_union;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(
    const int voxel_idx,
    const DataType& data)
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    block_data_[voxel_idx] = data;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(
    const Eigen::Vector3i& voxel_coord,
    const DataType& data)
{
    setData(getVoxelIdx(voxel_coord, current_scale), data);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(
    const Eigen::Vector3i& voxel_coord,
    const int scale,
    const DataType& data)
{
    setData(getVoxelIdx(voxel_coord, scale), data);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setDataUnion(
    const DataUnion& data_union)
{
    assert(data_union.data_idx >= 0);
    assert(static_cast<size_t>(data_union.data_idx) < block_data_.size());
    assert(data_union.prop_data_idx >= 0);
    assert(static_cast<size_t>(data_union.prop_data_idx) < block_past_data_.size());
    block_data_[data_union.data_idx] = data_union.data;
    block_past_data_[data_union.prop_data_idx] = data_union.past_data;
}


/// Multi-res occupancy

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes(
    const DataType init_data) :
        init_data_(init_data)
{
    const int num_voxels_at_scale = 1;
    DataType* data_at_scale = new DataType[num_voxels_at_scale];
    initialiseData(data_at_scale, num_voxels_at_scale);
    block_data_.push_back(data_at_scale);
    block_min_data_.push_back(data_at_scale);
    block_max_data_.push_back(data_at_scale);
    curr_data_ = block_data_.back();
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::~BlockMultiRes()
{
    for (DataType* data_at_scale : block_data_) {
        delete[] data_at_scale;
    }

    // Avoid double free as the last element of block_min_data_ is the same as the last element of
    // block_data_.
    if (!block_min_data_.empty()) {
        block_min_data_.pop_back();
    }
    for (DataType* min_data_at_scale : block_min_data_) {
        delete[] min_data_at_scale;
    }

    // Avoid double free as the last element of block_max_data_ is the same as the last element of
    // block_data_.
    if (!block_max_data_.empty()) {
        block_max_data_.pop_back();
    }
    for (DataType* max_data_at_scale : block_max_data_) {
        delete[] max_data_at_scale;
    }

    // If buffer_scale_ >= min_scale then buffer_data_ will contain the same value as some element
    // of block_data_ which has already been deallocated.
    if (buffer_data_ && buffer_scale_ < min_scale) {
        delete[] buffer_data_;
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
int BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getVoxelIdx(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    assert(scale >= 0);
    assert(scale <= max_scale);
    const Eigen::Vector3i voxel_offset = (voxel_coord - underlying()->coord) / (1 << scale);
    const int size_at_scale = BlockSize >> scale;
    return voxel_offset.x() + voxel_offset.y() * size_at_scale
        + voxel_offset.z() * math::sq(size_at_scale);
}



/// Get data at current scale

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord)
{
    return block_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



/// Get data at current scale or coarser

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



/// Get data at scale

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_data_[max_scale - scale][voxel_offset.x() + voxel_offset.y() * size_at_scale
                                              + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_data_[max_scale - scale][voxel_offset.x() + voxel_offset.y() * size_at_scale
                                              + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_min_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord)
{
    return block_min_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_min_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_min_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_min_data_[max_scale - scale][voxel_offset.x()
                                                  + voxel_offset.y() * size_at_scale
                                                  + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMinData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_min_data_[max_scale - scale][voxel_offset.x()
                                                  + voxel_offset.y() * size_at_scale
                                                  + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_max_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord)
{
    return block_max_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_max_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_max_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_max_data_[max_scale - scale][voxel_offset.x()
                                                  + voxel_offset.y() * size_at_scale
                                                  + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data_;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_max_data_[max_scale - scale][voxel_offset.x()
                                                  + voxel_offset.y() * size_at_scale
                                                  + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setData(
    const Eigen::Vector3i& voxel_coord,
    const DataType& data)
{
    block_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)] = data;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setData(
    const Eigen::Vector3i& voxel_coord,
    const int scale,
    const DataType& data)
{
    block_data_[max_scale - scale][getVoxelIdx(voxel_coord, scale)] = data;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setMaxData(
    const Eigen::Vector3i& voxel_coord,
    const DataType& data)
{
    block_max_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)] = data;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setMaxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale,
    const DataType& data)
{
    block_max_data_[max_scale - scale][getVoxelIdx(voxel_coord, scale)] = data;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::allocateDownTo(
    const int new_min_scale)
{
    assert(new_min_scale >= 0);
    assert(new_min_scale <= max_scale);
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(new_min_scale)) {
        block_min_data_.pop_back();
        block_max_data_.pop_back();
        int size_at_scale = BlockSize >> (max_scale - (block_data_.size() - 1));
        int num_voxels_at_scale = math::cu(size_at_scale);
        DataType* min_data_at_scale = new DataType[num_voxels_at_scale];
        DataType* max_data_at_scale = new DataType[num_voxels_at_scale];
        DataType* data_at_scale = block_data_[block_data_.size() - 1];
        std::copy(data_at_scale,
                  data_at_scale + num_voxels_at_scale,
                  min_data_at_scale); ///<< Copy init content.
        std::copy(data_at_scale,
                  data_at_scale + num_voxels_at_scale,
                  max_data_at_scale); ///<< Copy init content.
        block_min_data_.push_back(min_data_at_scale);
        block_max_data_.push_back(max_data_at_scale);

        for (int scale = max_scale - block_data_.size(); scale >= new_min_scale; scale--) {
            int size_at_scale = BlockSize >> scale;
            int num_voxels_at_scale = math::cu(size_at_scale);

            if (scale == new_min_scale) {
                DataType* data_at_scale = new DataType[num_voxels_at_scale];
                initialiseData(data_at_scale, num_voxels_at_scale);
                block_data_.push_back(data_at_scale);
                block_min_data_.push_back(
                    data_at_scale); ///<< Mean and min data are the same at the min scale.
                block_max_data_.push_back(
                    data_at_scale); ///<< Mean and max data are the same at the min scale.
            }
            else {
                DataType* data_at_scale = new DataType[num_voxels_at_scale];
                DataType* min_data_at_scale = new DataType[num_voxels_at_scale];
                DataType* max_data_at_scale = new DataType[num_voxels_at_scale];
                initialiseData(data_at_scale, num_voxels_at_scale);
                block_data_.push_back(data_at_scale);
                std::copy(data_at_scale,
                          data_at_scale + num_voxels_at_scale,
                          min_data_at_scale); ///<< Copy init content.
                std::copy(data_at_scale,
                          data_at_scale + num_voxels_at_scale,
                          max_data_at_scale); ///<< Copy init content.
                block_min_data_.push_back(min_data_at_scale);
                block_max_data_.push_back(max_data_at_scale);
            }
        }

        current_scale = new_min_scale;
        min_scale = new_min_scale;
        curr_data_ = block_data_[max_scale - new_min_scale];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::deleteUpTo(
    const int new_min_scale)
{
    assert(new_min_scale >= 0);
    assert(new_min_scale <= max_scale);
    if (min_scale == -1 || min_scale >= new_min_scale)
        return;

    auto& data_at_scale = block_data_[max_scale - min_scale];
    delete[] data_at_scale;
    block_data_.pop_back();
    block_min_data_
        .pop_back(); ///<< Avoid double free as the min scale data points to the same data.
    block_max_data_
        .pop_back(); ///<< Avoid double free as the min scale data points to the same data.

    for (int scale = min_scale + 1; scale < new_min_scale; scale++) {
        // Delete mean data
        data_at_scale = block_data_[max_scale - scale];
        delete[] data_at_scale;
        block_data_.pop_back();

        // Delete min data
        auto& min_data_at_scale = block_min_data_[max_scale - scale];
        delete[] min_data_at_scale;
        block_min_data_.pop_back();

        // Delete max data
        auto& max_data_at_scale = block_max_data_[max_scale - scale];
        delete[] max_data_at_scale;
        block_max_data_.pop_back();
    }

    // Replace min data at min scale with same as mean data.
    auto& min_data_at_scale = block_min_data_[max_scale - new_min_scale];
    delete[] min_data_at_scale;
    block_min_data_.pop_back();
    block_min_data_.push_back(block_data_[max_scale - new_min_scale]);

    // Replace max data at min scale with same as mean data.
    auto& max_data_at_scale = block_max_data_[max_scale - new_min_scale];
    delete[] max_data_at_scale;
    block_max_data_.pop_back();
    block_max_data_.push_back(block_data_[max_scale - new_min_scale]);

    min_scale = new_min_scale;
}


template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::incrCurrObservedCount(
    bool do_increment)
{
    if (do_increment) {
        curr_observed_count_++;
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetCurrCount()
{
    curr_integr_count_ = 0;
    curr_observed_count_ = 0;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::initCurrCout()
{
    if (init_data_.field.observed) {
        int size_at_scale = BlockSize >> current_scale;
        int num_voxels_at_scale = math::cu(size_at_scale);
        curr_integr_count_ = init_data_.field.weight;
        curr_observed_count_ = num_voxels_at_scale;
    }
    else {
        resetCurrCount();
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::incrBufferIntegrCount(
    const bool do_increment)
{
    if (do_increment
        || buffer_observed_count_ * math::cu(1 << buffer_scale_)
            >= 0.90 * curr_observed_count_ * math::cu(1 << current_scale)) {
        buffer_integr_count_++;
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::
    incrBufferObservedCount(const bool do_increment)
{
    if (do_increment) {
        buffer_observed_count_++;
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetBufferCount()
{
    buffer_integr_count_ = 0;
    buffer_observed_count_ = 0;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetBuffer()
{
    if (buffer_scale_ < current_scale) {
        delete[] buffer_data_;
    }
    buffer_data_ = nullptr;
    buffer_scale_ = -1;
    resetBufferCount();
}

template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::initBuffer(
    const int buffer_scale)
{
    assert(buffer_scale >= 0);
    assert(buffer_scale <= max_scale);
    resetBuffer();

    buffer_scale_ = buffer_scale;

    if (buffer_scale < current_scale) {
        // Initialise all data to init data.
        const int size_at_scale = BlockSize >> buffer_scale;
        const int num_voxels_at_scale = math::cu(size_at_scale);
        buffer_data_ = new DataType[num_voxels_at_scale]; ///<< Data must still be initialised.
    }
    else {
        buffer_data_ = block_data_[max_scale - buffer_scale_];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
bool BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::switchData()
{
    if (buffer_integr_count_ >= 20
        && buffer_observed_count_ * math::cu(1 << buffer_scale_)
            >= 0.9 * curr_observed_count_ * math::cu(1 << current_scale)) { // TODO: Find threshold

        /// !!! We'll switch !!!
        if (buffer_scale_ < current_scale) { ///<< Switch to finer scale.
            block_data_.push_back(buffer_data_);
            block_min_data_.push_back(buffer_data_); ///< Share data at finest scale.
            block_max_data_.push_back(buffer_data_); ///< Share data at finest scale.

            /// Add allocate data for the scale that mean and max data shared before.
            const int size_at_scale = BlockSize >> (buffer_scale_ + 1);
            const int num_voxels_at_scale = math::cu(size_at_scale);
            block_min_data_[max_scale - (buffer_scale_ + 1)] =
                new DataType[num_voxels_at_scale]; ///<< Data must still be initialised.
            block_max_data_[max_scale - (buffer_scale_ + 1)] =
                new DataType[num_voxels_at_scale]; ///<< Data must still be initialised.
        }
        else { ///<< Switch to coarser scale.
            deleteUpTo(buffer_scale_);
        }

        /// Update observed state
        const int size_at_buffer_scale = BlockSize >> buffer_scale_;
        const int num_voxels_at_buffer_scale = math::cu(size_at_buffer_scale);

        int missed_observed_count = 0;
        for (int voxel_idx = 0; voxel_idx < num_voxels_at_buffer_scale; voxel_idx++) {
            DataType& data = buffer_data_[voxel_idx];
            if (data.field.weight > 0 && !data.field.observed) {
                data.field.observed = true;
                buffer_observed_count_++;
                missed_observed_count++;
            }
        }

        current_scale = buffer_scale_;
        min_scale = buffer_scale_;

        curr_data_ = buffer_data_;
        curr_integr_count_ = buffer_integr_count_;
        curr_observed_count_ = buffer_observed_count_;
        buffer_data_ = nullptr;
        buffer_scale_ = -1;
        resetBufferCount();
        return true;
    }
    return false;
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::bufferData(
    const Eigen::Vector3i& voxel_coord) const
{
    return buffer_data_[getVoxelIdx(voxel_coord, buffer_scale_)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::bufferData(
    const Eigen::Vector3i& voxel_coord)
{
    return buffer_data_[getVoxelIdx(voxel_coord, buffer_scale_)];
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::blockDataAtScale(
    const int scale)
{
    assert(scale >= 0);
    assert(scale <= max_scale);
    if (scale < min_scale) {
        return nullptr;
    }
    else {
        return block_data_[max_scale - scale];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::blockMinDataAtScale(
    const int scale)
{
    assert(scale >= 0);
    assert(scale <= max_scale);
    if (scale < min_scale) {
        return nullptr;
    }
    else {
        return block_min_data_[max_scale - scale];
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::blockMaxDataAtScale(
    const int scale)
{
    assert(scale >= 0);
    assert(scale <= max_scale);
    if (scale < min_scale) {
        return nullptr;
    }
    else {
        return block_max_data_[max_scale - scale];
    }
}



template<typename DataT, Res ResT, int BlockSize>
Block<DataT, ResT, BlockSize>::Block(Node<DataT, ResT>* parent_ptr,
                                     const int child_idx,
                                     const DataT init_data) :
        OctantBase(parent_ptr->coord
                       + BlockSize
                           * Eigen::Vector3i((1 & child_idx) > 0,
                                             (2 & child_idx) > 0,
                                             (4 & child_idx) > 0),
                   true,
                   parent_ptr),
        std::conditional<
            ResT == Res::Single,
            BlockSingleRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>,
            BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::type(init_data)
{
    assert(BlockSize == (parent_ptr->size >> 1));
}



} // namespace se

#endif // SE_BLOCK_IMPL_HPP
