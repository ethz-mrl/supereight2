/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_BLOCK_HPP
#define SE_BLOCK_HPP

namespace se {

// Forward declare Node to avoid depending on the order of includes.
template<typename DataT, Res ResT>
class Node;

// Forward declare Block to allow accessing it from BlockData.
template<typename DataT, Res ResT, int BlockSize>
class Block;



/** Contains se::Data stored in se::Block and appropriate methods. Partial template specilization is
 * used so that se::Block doesn't contain unnecessary data. This non-specialized version contains an
 * array of se::Data with size BlockSize³, as used for se::Res::Single se::Field::TSDF.
 */
template<typename DataT, Res ResT, int BlockSize>
struct BlockData {
    typedef DataT DataType;

    /** The maximum scale of the stored data. */
    static constexpr int max_scale = 0;
    /** The minimum scale the data has been updated at. */
    static constexpr int min_scale = 0;
    /** The scale the data was last updated at. */
    static constexpr int current_scale = 0;

    /** Return a reference to the data at voxel coordinates \p voxel_coord. */
    DataType& data(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& data(const Eigen::Vector3i& voxel_coord) const;

    /** Return a reference to the data at linear index \p voxel_idx ∈ [0, BlockSize³ - 1]. */
    DataType& data(const int voxel_idx);
    /** \constoverload */
    const DataType& data(const int voxel_idx) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    BlockData(const DataType& init_data = DataType());

    private:
    std::array<DataType, math::cu(BlockSize)> data_;

    /** Cast to the derived class se::Block to allow accessing its members. */
    const Block<DataType, ResT, BlockSize>* derived() const;
};



/** Specialization of se::BlockData for se::Res::Multi se::Field::TSDF. It contains an array holding
 * the se::Data for all scales. */
template<Colour ColB, Id IdB, int BlockSize>
struct BlockData<Data<Field::TSDF, ColB, IdB>, Res::Multi, BlockSize> {
    typedef Data<Field::TSDF, ColB, IdB> DataType;

    /** Contains data from a previous point in time which is used to compute changes over time for
     * delta down-propagation.
     */
    typedef DataType PastDataType;

    /** Simplifies access to current and past data at the same voxel coordinates. */
    struct DataUnion {
        const Eigen::Vector3i coord;
        const int scale;
        DataType& data;
        PastDataType& past_data;
        const int data_idx;
    };

    /** The maximum scale of the stored data. */
    static constexpr int max_scale = math::log2_const(BlockSize);
    /** The minimum scale the data has been updated at. -1 if no update has been performed. */
    int min_scale = -1;
    /** The scale the data was last updated at. -1 if no update has been performed. */
    int current_scale = -1;

    /** Return a reference to the data at the current scale at voxel coordinates \p voxel_coord. */
    DataType& data(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& data(const Eigen::Vector3i& voxel_coord) const;

    /** Return a reference to the data at scale \p scale at voxel coordinates \p voxel_coord. */
    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale);
    /** \constoverload */
    const DataType& data(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Return a reference to the data at scale \p scale_desired or coarser at voxel coordinates \p
     * voxel_coord. The actual scale of the data is written to \p scale_returned and is no finer
     * than current_scale.
     */
    DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);
    /** \constoverload */
    const DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned) const;

    /** Return a reference to the data at linear index \p voxel_idx ∈ [0, num_voxels_ - 1]. */
    DataType& data(const int voxel_idx);
    /** \constoverload */
    const DataType& data(const int voxel_idx) const;

    /** Return the current and past data at scale \p scale at voxel coordinates \p voxel_coord. */
    DataUnion dataUnion(const Eigen::Vector3i& voxel_coord, const int scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    BlockData(const DataType& init_data = DataType());

    private:
    /** Return the total number of voxels in all scales. */
    static constexpr int compute_num_voxels()
    {
        size_t voxel_count = 0;
        for (int size_at_scale = BlockSize; size_at_scale > 0; size_at_scale /= 2) {
            voxel_count += math::cu(size_at_scale);
        }
        return voxel_count;
    }

    /** Return the edge length in voxels at each scale. */
    static constexpr std::array<int, max_scale + 1> compute_size_at_scales()
    {
        std::array<int, max_scale + 1> size_at_scales{};
        for (int size_at_scale = BlockSize, scale = 0; size_at_scale > 0;
             size_at_scale /= 2, scale++) {
            size_at_scales[scale] = size_at_scale;
        }
        return size_at_scales;
    }

    /** Return the offsets in elements into data_ that the data for each scale begins at. */
    static constexpr std::array<int, max_scale + 1> compute_scale_offsets()
    {
        std::array<int, max_scale + 1> scale_offsets{0};
        for (int size_at_scale = BlockSize, scale = 1; size_at_scale > 1;
             size_at_scale /= 2, scale++) {
            scale_offsets[scale] = scale_offsets[scale - 1] + math::cu(size_at_scale);
        }
        return scale_offsets;
    }

    static constexpr int num_voxels_ = compute_num_voxels();
    static constexpr std::array<int, max_scale + 1> size_at_scales_ = compute_size_at_scales();
    static constexpr std::array<int, max_scale + 1> scale_offsets_ = compute_scale_offsets();

    std::array<DataType, num_voxels_> data_;
    std::array<PastDataType, num_voxels_> past_data_;

    /** Return the index into data_ for the data at voxel coordinates \p voxel_coord and scale \p
     * scale .
     */
    int voxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Cast to the derived class se::Block to allow accessing its members. */
    const Block<DataType, Res::Multi, BlockSize>* derived() const;
};



/** Specialization of se::BlockData for se::Field::Occupancy. It contains mean, minimum, and maximum
 * data up to some scale. */
template<Colour ColB, Id IdB, int BlockSize>
struct BlockData<Data<Field::Occupancy, ColB, IdB>, Res::Multi, BlockSize> {
    /** The type of the stored data. */
    typedef Data<Field::Occupancy, ColB, IdB> DataType;

    /** The maximum scale of the stored data. */
    static constexpr int max_scale = math::log2_const(BlockSize);
    /** The minimum scale the data has been updated at. -1 if no update has been performed. */
    int min_scale = -1;
    /** The scale the data was last updated at. */
    int current_scale = max_scale;
    /** The data the block was initialised with. */
    DataType init_data;


    /** \name Data access */
    /**@{*/
    /** Return a reference to the data at the current scale at voxel coordinates \p voxel_coord. */
    DataType& data(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& data(const Eigen::Vector3i& voxel_coord) const;

    /** Return a reference to the data at scale \p scale at voxel coordinates \p voxel_coord. */
    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale);
    /** \constoverload */
    const DataType& data(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Return a reference to the data at scale \p scale_desired or coarser at voxel coordinates \p
     * voxel_coord. The actual scale of the data is written to \p scale_returned and is no smaller
     * than current_scale.
     */
    DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);
    /** \constoverload */
    const DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned) const;

    /** Return a const reference to the block's data at the coarsest scale. */
    const DataType& data() const;

    /** Return a pointer to the block data array for scale \p scale. Return `nullptr` if \p scale
     * is smaller than the finest allocated scale.
     */
    DataType* blockDataAtScale(const int scale);
    /**@}*/


    /** \name Minimum data access */
    /**@{*/
    /** Return a reference to the minimum data at the current scale at voxel coordinates \p
     * voxel_coord.
     */
    DataType& minData(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& minData(const Eigen::Vector3i& voxel_coord) const;

    /** Return a reference to the minimum data at scale \p scale at voxel coordinates \p
     * voxel_coord.
     */
    DataType& minData(const Eigen::Vector3i& voxel_coord, const int scale);
    /** \constoverload */
    const DataType& minData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Return a reference to the minimum data at scale \p scale or coarser at voxel coordinates \p
     * voxel_coord. The actual scale of the data is written to \p scale_returned and is no smaller
     * than current_scale.
     */
    DataType&
    minData(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);
    /** \constoverload */
    const DataType&
    minData(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned) const;

    /** Return a const reference to the block's minimum data at the coarsest scale. */
    const DataType& minData() const;

    /** Return a pointer to the block minimum data array for scale \p scale. Return `nullptr` if \p
     * scale is smaller than the finest allocated scale.
     */
    DataType* blockMinDataAtScale(const int scale);
    /**@}*/


    /** \name Maximum data access */
    /**@{*/
    DataType& maxData(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& maxData(const Eigen::Vector3i& voxel_coord) const;

    DataType& maxData(const Eigen::Vector3i& voxel_coord, const int scale);
    /** \constoverload */
    const DataType& maxData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Return a reference to the maximum data at scale \p scale or coarser at voxel coordinates \p
     * voxel_coord. The actual scale of the data is written to \p scale_returned and is no smaller
     * than current_scale.
     */
    DataType&
    maxData(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);
    /** \constoverload */
    const DataType&
    maxData(const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned) const;

    /** Return a const reference to the block's maximum data at the coarsest scale. */
    const DataType& maxData() const;

    /** Return a pointer to the block maximum data array for scale \p scale. Return `nullptr` if \p
     * scale is smaller than the finest allocated scale.
     */
    DataType* blockMaxDataAtScale(const int scale);
    /**@}*/


    /** \name Scale (de)allocation */
    /**@{*/
    /** Allocate the mip-mapped scales down to scale \p new_min_scale. */
    void allocateDownTo(const int new_min_scale = 0);

    /** Delete the mip-mapped scales up to scale \p new_min_scale. */
    void deleteUpTo(const int new_min_scale);
    /**@}*/


    /** \name Buffer management */
    /**@{*/
    /** Return the number of integrations at the current scale. */
    size_t currIntegrCount() const;

    /** Return the number of observed voxels at the current scale. */
    size_t currObservedCount() const;

    /** Increment the number of integrations at the current scale by 1. */
    void incrCurrIntegrCount();

    /** Increment the number of observed voxels in at the current scale by 1 if \p do_increment is
     * true.
     */
    void incrCurrObservedCount(bool do_increment = true);

    /** When a block is initialised from an observed block (i.e. init_data.observed == true), set
     * the current observed count to all voxels observed and the integration count to the nodes
     * value. Otherwise reset the current count.
     */
    void initCurrCount();

    /** Reset the current integration and observation counts to 0. */
    void resetCurrCount();

    /** Return the buffer integration scale. */
    int bufferScale() const;

    /** Return the number of integrations in the buffer. */
    size_t bufferIntegrCount() const;

    /** Return the number of observed voxels in the buffer. */
    size_t bufferObservedCount() const;

    /** Increment the number of integrations in the buffer by 1 if \p do_increment is true.
     * Typically \p do_increment is true if the scale normalised number of observations at the
     * buffer scale >= 95% of the observations at the current scale.
     */
    void incrBufferIntegrCount(const bool do_increment = true);

    /** Increment the number of observed voxels in the buffer by 1 if \p do_increment is true. */
    void incrBufferObservedCount(const bool do_increment = true);

    /** Initialise the buffer at scale \p buffer_scale. */
    void initBuffer(const int buffer_scale);

    /** Reset the buffer integration and observation counts to 0. */
    void resetBufferCount();

    /** Reset the buffer and free any associated data. */
    void resetBuffer();

    /** Switch the buffer data into the block data if possible and return whether the switch was
     * made. */
    bool switchData();

    /** Return a reference to the buffer data at voxel coordinates \p voxel_coord. */
    DataType& bufferData(const Eigen::Vector3i& voxel_coord);
    /** \constoverload */
    const DataType& bufferData(const Eigen::Vector3i& voxel_coord) const;

    /** Return a reference to the buffer data at linear index \p voxel_idx. The maximum value of \p
     * voxel_idx depends on the buffer scale. */
    DataType& bufferData(const int voxel_idx);
    /** \constoverload */
    const DataType& bufferData(const int voxel_idx) const;
    /**@}*/

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    BlockData(const DataType init_data = DataType());
    ~BlockData();

    private:
    /** The block data. Each element is a pointer to the data at some scale, starting from the
     * coarsest. */
    std::vector<DataType*> data_;
    /** The minimum block data. Organised like data_. */
    std::vector<DataType*> min_data_;
    /** The maximum block data. Organised like data_. */
    std::vector<DataType*> max_data_;

    /** The number of integrations at the current scale. */
    size_t curr_integr_count_;
    /** The number of observed voxels at the current scale. */
    size_t curr_observed_count_;

    /** The buffer data. Rather than switching directly to a different integration scale once the
     * integration scale computation recommends a different scale, data is continued to be
     * integrated at the current scale and additionally into a buffer at the recommended scale.
     *
     * - recommended scale == current scale:
     *       buffer_data_ is `nullptr`
     * - recommended scale < current scale:
     *       The buffer_data_ points to a independently allocated array of voxel data. The data is
     *       initialised with the parent data at the current integration scale. Once the scale
     *       changes the data is inserted into the data_ and max_data_ vector.
     * - recommended scale > current scale:
     *       The buffer_data_ points to according scale in the data_ vector. The data integration
     *       starts from the mean up-propagated value. Up until the recommened scale > current scale
     *       the mean up-propagation starts from the recommened scale such that the data is not
     *       overwritten by the up-propagation from the current scale. However the max
     *       up-propagation continues from the current integration scale. Once the scale changes the
     *       current_data_ and current_scale_ is set to the buffer setup, the finest scale in the
     *       data_ and max_data_ is deleted and the new finest scales in the buffers adjusted
     *       accordingly.
     *
     * \note  The recommended scale can only differ by ±1 scale from the current scale. The
     * overhead of integrating at two different scales is insignificant compared to switching
     * immediately as the double integration only happens in areas where the recommended integration
     * scale changed and stops as soon as the criteria for switching to the finer or coarser scale.
     */
    DataType* buffer_data_ = nullptr;
    /** The buffer scale. */
    int buffer_scale_ = -1;
    /** The number of integrations at the buffer scale.
     * \note Is only incremented when 95% of the current observations are reached.
     */
    size_t buffer_integr_count_;
    /** The number of observed voxels in the buffer. */
    size_t buffer_observed_count_;

    /** Return the index into the respective element of data_/min_data_/max_data_ for the data at
     * voxel coordinates \p voxel_coord and scale \p scale .
     */
    int voxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /** Cast to the derived class se::Block to allow accessing its members. */
    const Block<DataType, Res::Multi, BlockSize>* derived() const;
};



/** A leaf node of an se::Octree.
 *
 * \tparam DataT     The type of data stored in the octree.
 * \tparam ResT      The value of se::Res for the octree.
 * \tparam BlockSize The edge length of se::Block in voxels.
 */
template<typename DataT, Res ResT, int BlockSize>
class Block : public OctantBase, public BlockData<DataT, ResT, BlockSize> {
    public:
    typedef DataT DataType;

    /** The edge length of the block in voxels. */
    static constexpr int size = BlockSize;
    /** The face area of the block in voxels. */
    static constexpr int size_sq = math::sq(BlockSize);
    /** The volume of the block in voxels. */
    static constexpr int size_cu = math::cu(BlockSize);

    /** Construct the child block of \p parent_ptr with index \p child_idx and initialize its data
     * at the coarsest scale with \p init_data. The value of \p child_idx must be in the interval
     * [0, 7] inclusive.
     */
    Block(Node<DataT, ResT>* parent_ptr, const int child_idx, const DataT init_data);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert(math::is_power_of_two(BlockSize));
};

} // namespace se

#include "impl/block_impl.hpp"

#endif // SE_BLOCK_HPP
