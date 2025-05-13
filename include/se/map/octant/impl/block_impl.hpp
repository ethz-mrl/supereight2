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

template<typename DataT, Res ResT, int BlockSize>
typename BlockData<DataT, ResT, BlockSize>::DataType&
BlockData<DataT, ResT, BlockSize>::data(const Eigen::Vector3i& voxel_coord)
{
    return const_cast<DataType&>(
        const_cast<const BlockData<DataT, ResT, BlockSize>*>(this)->data(voxel_coord));
}

template<typename DataT, Res ResT, int BlockSize>
const typename BlockData<DataT, ResT, BlockSize>::DataType&
BlockData<DataT, ResT, BlockSize>::data(const Eigen::Vector3i& voxel_coord) const
{
    // Compute a column-major index from block-relative voxel coordinates:
    // int voxel_idx = x + (y * BlockSize) + (z * BlockSize * BlockSize);
    static const Eigen::Vector3i column_major_coeffs(1, BlockSize, math::sq(BlockSize));
    const int voxel_idx = (voxel_coord - derived()->coord).dot(column_major_coeffs);
    return data(voxel_idx);
}

template<typename DataT, Res ResT, int BlockSize>
typename BlockData<DataT, ResT, BlockSize>::DataType&
BlockData<DataT, ResT, BlockSize>::data(const int voxel_idx)
{
    return const_cast<DataType&>(
        const_cast<const BlockData<DataT, ResT, BlockSize>*>(this)->data(voxel_idx));
}

template<typename DataT, Res ResT, int BlockSize>
const typename BlockData<DataT, ResT, BlockSize>::DataType&
BlockData<DataT, ResT, BlockSize>::data(const int voxel_idx) const
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < data_.size());
    return data_[voxel_idx];
}

template<typename DataT, Res ResT, int BlockSize>
BlockData<DataT, ResT, BlockSize>::BlockData(const DataType& init_data)
{
    // XXX: The std::array constructor will default-initialize all std::array elements. The
    // following .fill() and init_data parameter are only necessary if we need initialization to
    // non-default values. Removing the .fill() will result in a (maybe insignificant) performance
    // increase.
    data_.fill(init_data);
}

template<typename DataT, Res ResT, int BlockSize>
const Block<DataT, ResT, BlockSize>* BlockData<DataT, ResT, BlockSize>::derived() const
{
    return static_cast<const Block<DataT, ResT, BlockSize>*>(this);
}



/// Multi-res Block ///

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
int BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::getVoxelIdx(
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(const int voxel_idx) const
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(const int voxel_idx)
{
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    return block_data_[voxel_idx];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord) const
{
    return data(voxel_coord, current_scale);
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord)
{
    return data(voxel_coord, current_scale);
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale_desired,
    int& scale_returned) const
{
    scale_returned = std::max(scale_desired, current_scale);
    return data(getVoxelIdx(voxel_coord, scale_returned));
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale_desired,
    int& scale_returned)
{
    scale_returned = std::max(scale_desired, current_scale);
    return data(getVoxelIdx(voxel_coord, scale_returned));
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    return data(getVoxelIdx(voxel_coord, scale));
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    return data(getVoxelIdx(voxel_coord, scale));
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::DataUnion
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::dataUnion(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    const int voxel_idx = getVoxelIdx(voxel_coord, scale);
    assert(voxel_idx >= 0);
    assert(static_cast<size_t>(voxel_idx) < block_data_.size());
    assert(static_cast<size_t>(voxel_idx) < block_past_data_.size());
    return DataUnion{
        voxel_coord,
        scale,
        block_data_[voxel_idx],
        block_past_data_[voxel_idx],
        voxel_idx,
    };
}

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT>::BlockMultiRes(
    const DataType init_data)
{
    block_data_.fill(init_data); // TODO: Verify that initialisation doesn't cause regression
}



/// Multi-res occupancy

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::BlockMultiRes(
    const DataType init_data) :
        init_data(init_data)
{
    const int num_voxels_at_scale = 1;
    DataType* data_at_scale = new DataType[num_voxels_at_scale];
    std::fill(data_at_scale, data_at_scale + num_voxels_at_scale, init_data);
    block_data_.push_back(data_at_scale);
    block_min_data_.push_back(data_at_scale);
    block_max_data_.push_back(data_at_scale);
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::~BlockMultiRes()
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
int BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::getVoxelIdx(
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

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord)
{
    return block_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



/// Get data at current scale or coarser

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



/// Get data at scale

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_data_[max_scale - scale][voxel_offset.x() + voxel_offset.y() * size_at_scale
                                              + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::data(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
    }
    else {
        Eigen::Vector3i voxel_offset = voxel_coord - underlying()->coord;
        voxel_offset = voxel_offset / (1 << scale);
        const int size_at_scale = BlockSize >> scale;
        return block_data_[max_scale - scale][voxel_offset.x() + voxel_offset.y() * size_at_scale
                                              + voxel_offset.z() * math::sq(size_at_scale)];
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_min_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord)
{
    return block_min_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_min_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_min_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::minData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord) const
{
    return block_max_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord)
{
    return block_max_data_[max_scale - current_scale][getVoxelIdx(voxel_coord, current_scale)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out) const
{
    scale_out = std::max(scale_in, current_scale);
    return block_max_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale_in,
    int& scale_out)
{
    scale_out = std::max(scale_in, current_scale);
    return block_max_data_[max_scale - scale_out][getVoxelIdx(voxel_coord, scale_out)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale) const
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::maxData(
    const Eigen::Vector3i& voxel_coord,
    const int scale)
{
    if (max_scale - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
        return init_data;
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::allocateDownTo(
    const int new_min_scale)
{
    assert(new_min_scale >= 0);
    assert(new_min_scale <= max_scale);
    if (new_min_scale >= current_scale) {
        return;
    }

    // The min/max data at the current minimum scale point to the mean data. Remove them to
    // create copies later.
    block_min_data_.pop_back();
    block_max_data_.pop_back();
    for (int scale = current_scale; scale >= new_min_scale; scale--) {
        const int size_at_scale = BlockSize >> scale;
        const int num_voxels_at_scale = math::cu(size_at_scale);
        // Set the mean data.
        DataType* data_at_scale;
        if (scale == current_scale) {
            // The mean data is already allocated at the current scale.
            data_at_scale = block_data_.back();
        }
        else {
            // Allocate new mean data for scales finer than the current.
            data_at_scale = new DataType[num_voxels_at_scale];
            // XXX: Should finer scales be initialized with the data of the current scale
            // instead of init_data?
            std::fill(data_at_scale, data_at_scale + num_voxels_at_scale, init_data);
            block_data_.push_back(data_at_scale);
        }
        // Set the min/max data.
        if (scale == new_min_scale) {
            // Set min/max data to the mean data at the minimum scale.
            block_min_data_.push_back(data_at_scale);
            block_max_data_.push_back(data_at_scale);
        }
        else {
            block_min_data_.push_back(new DataType[num_voxels_at_scale]);
            block_max_data_.push_back(new DataType[num_voxels_at_scale]);
            // Initialize min/max data at intermediate scales with the mean data.
            std::copy(data_at_scale, data_at_scale + num_voxels_at_scale, block_min_data_.back());
            std::copy(data_at_scale, data_at_scale + num_voxels_at_scale, block_max_data_.back());
        }
    }

    current_scale = new_min_scale;
    min_scale = new_min_scale;
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::deleteUpTo(
    const int new_min_scale)
{
    assert(new_min_scale >= 0);
    assert(new_min_scale <= max_scale);
    if (new_min_scale <= min_scale || min_scale == -1) {
        return;
    }

    for (int scale = min_scale; scale < new_min_scale; scale++) {
        delete[] block_data_.back();
        block_data_.pop_back();
        // Don't delete the min/max data at the minimum scale to avoid a double-free, since it is
        // the same as the mean data.
        if (scale > min_scale) {
            delete[] block_min_data_.back();
            delete[] block_max_data_.back();
        }
        block_min_data_.pop_back();
        block_max_data_.pop_back();
    }

    // Set the min/max data at the new minimum scale to be the same as the mean data.
    delete[] block_min_data_.back();
    delete[] block_max_data_.back();
    block_min_data_.pop_back();
    block_max_data_.pop_back();
    block_min_data_.push_back(block_data_.back());
    block_max_data_.push_back(block_data_.back());

    current_scale = new_min_scale;
    min_scale = new_min_scale;
}


template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::incrCurrObservedCount(
    bool do_increment)
{
    if (do_increment) {
        curr_observed_count_++;
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::resetCurrCount()
{
    curr_integr_count_ = 0;
    curr_observed_count_ = 0;
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::initCurrCout()
{
    if (init_data.field.observed) {
        int size_at_scale = BlockSize >> current_scale;
        int num_voxels_at_scale = math::cu(size_at_scale);
        curr_integr_count_ = init_data.field.weight;
        curr_observed_count_ = num_voxels_at_scale;
    }
    else {
        resetCurrCount();
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::incrBufferIntegrCount(
    const bool do_increment)
{
    if (do_increment
        || buffer_observed_count_ * math::cu(1 << buffer_scale_)
            >= 0.90 * curr_observed_count_ * math::cu(1 << current_scale)) {
        buffer_integr_count_++;
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::incrBufferObservedCount(
    const bool do_increment)
{
    if (do_increment) {
        buffer_observed_count_++;
    }
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::resetBufferCount()
{
    buffer_integr_count_ = 0;
    buffer_observed_count_ = 0;
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::resetBuffer()
{
    if (buffer_scale_ < current_scale) {
        delete[] buffer_data_;
    }
    buffer_data_ = nullptr;
    buffer_scale_ = -1;
    resetBufferCount();
}

template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
void BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::initBuffer(
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
bool BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::switchData()
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
            current_scale = buffer_scale_;
            min_scale = buffer_scale_;
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

        curr_integr_count_ = buffer_integr_count_;
        curr_observed_count_ = buffer_observed_count_;
        buffer_data_ = nullptr;
        buffer_scale_ = -1;
        resetBufferCount();
        return true;
    }
    return false;
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
const typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::bufferData(
    const Eigen::Vector3i& voxel_coord) const
{
    return buffer_data_[getVoxelIdx(voxel_coord, buffer_scale_)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::bufferData(
    const Eigen::Vector3i& voxel_coord)
{
    return buffer_data_[getVoxelIdx(voxel_coord, buffer_scale_)];
}



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::blockDataAtScale(
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::blockMinDataAtScale(
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



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
typename BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT>::blockMaxDataAtScale(
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
            BlockData<DataT, ResT, BlockSize>,
            BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::type(init_data)
{
    assert(BlockSize == (parent_ptr->size >> 1));
}



} // namespace se

#endif // SE_BLOCK_IMPL_HPP
