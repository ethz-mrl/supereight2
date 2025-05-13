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



template<typename DataT, int BlockSize, typename DerivedT>
class BlockMultiRes {
};



template<Field FldT, Colour ColB, Id IdB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<FldT, ColB, IdB>, BlockSize, DerivedT> {
};



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<Field::TSDF, ColB, IdB>, BlockSize, DerivedT> {
    public:
    typedef Data<Field::TSDF, ColB, IdB> DataType;

    /** Contains data from a previous point in time which is used to compute changes over time for
     * delta down-propagation.
     */
    typedef DataType PastDataType;

    struct DataUnion {
        const Eigen::Vector3i coord;
        const int scale;
        DataType& data;
        PastDataType& past_data;
        const int data_idx;
    };

    static constexpr int max_scale = math::log2_const(BlockSize);
    int min_scale = -1;
    int current_scale = -1;

    DataType& data(const Eigen::Vector3i& voxel_coord);
    const DataType& data(const Eigen::Vector3i& voxel_coord) const;

    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale);
    const DataType& data(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);
    const DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& data(const int voxel_idx);
    const DataType& data(const int voxel_idx) const;

    DataUnion dataUnion(const Eigen::Vector3i& voxel_coord, const int scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    BlockMultiRes(const DataType init_data = DataType());

    private:
    static constexpr int compute_num_voxels()
    {
        size_t voxel_count = 0;
        for (int size_at_scale = BlockSize; size_at_scale > 0; size_at_scale /= 2) {
            voxel_count += math::cu(size_at_scale);
        }
        return voxel_count;
    }

    static constexpr std::array<int, max_scale + 1> compute_size_at_scales()
    {
        std::array<int, max_scale + 1> size_at_scales{};
        for (int size_at_scale = BlockSize, scale = 0; size_at_scale > 0;
             size_at_scale /= 2, scale++) {
            size_at_scales[scale] = size_at_scale;
        }
        return size_at_scales;
    }

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

    int voxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;

    const DerivedT* derived() const
    {
        return static_cast<const DerivedT*>(this);
    }
};



template<Colour ColB, Id IdB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<Field::Occupancy, ColB, IdB>, BlockSize, DerivedT> {
    public:
    typedef Data<Field::Occupancy, ColB, IdB> DataType;

    static constexpr int max_scale = math::log2_const(BlockSize);
    int min_scale = -1;
    int current_scale = max_scale;
    DataType init_data;

    BlockMultiRes(const DataType init_data = DataType());

    BlockMultiRes(const Block<Data<Field::Occupancy, ColB, IdB>, Res::Multi, BlockSize>& block);

    void operator=(const Block<Data<Field::Occupancy, ColB, IdB>, Res::Multi, BlockSize>& block);

    ~BlockMultiRes();

    int getVoxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;



    /// Get data at current scale

    const DataType& data(const Eigen::Vector3i& voxel_coord) const;

    DataType& data(const Eigen::Vector3i& voxel_coord);

    /// Get data at current scale or coarser

    const DataType&
    data(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    /// Get data at scale

    const DataType& data(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& data(const Eigen::Vector3i& voxel_coord, const int scale);

    const DataType& minData(const Eigen::Vector3i& voxel_coord) const;

    DataType& minData(const Eigen::Vector3i& voxel_coord);

    const DataType&
    minData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& minData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    const DataType& minData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& minData(const Eigen::Vector3i& voxel_coord, const int scale);

    const DataType& maxData(const Eigen::Vector3i& voxel_coord) const;

    DataType& maxData(const Eigen::Vector3i& voxel_coord);

    const DataType&
    maxData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& maxData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    const DataType& maxData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& maxData(const Eigen::Vector3i& voxel_coord, const int scale);

    /**
     * \brief Allocate the mip-mapped scales down to 'new_min_scale'.
     */
    void allocateDownTo(const int new_min_scale = 0);

    /**
     * \brief Delete the mip-mapped scales up to 'new_min_scale'.
     */
    void deleteUpTo(const int new_min_scale);

    /**
     * \brief Get the block's data at the coarsest scale.
     *
     * \return The block's data at the coarsest scale
     */
    const DataType& data() const
    {
        assert(!block_data_.empty());
        assert(block_data_.front());
        return block_data_[0][0];
    }

    /**
     * \brief Get the block's min data at the coarsest scale.
     *
     * \return The block's min data at the coarsest scale
     */
    const DataType& minData() const
    {
        assert(!block_min_data_.empty());
        assert(block_min_data_.front());
        return block_min_data_[0][0];
    }

    /**
     * \brief Get the block's max data at the coarsest scale.
     *
     * \return The block's max data at the coarsest scale
     */
    const DataType& maxData() const
    {
        assert(!block_max_data_.empty());
        assert(block_max_data_.front());
        return block_max_data_[0][0];
    }

    /**
     * \brief Get the number of integrations at the current scale.
     */
    size_t currIntegrCount() const
    {
        return curr_integr_count_;
    }

    /**
     * \brief Get the number of observed voxels at the current scale.
     */
    size_t currObservedCount() const
    {
        return curr_observed_count_;
    }

    /**
     * \brief Increment the number of integrations at the current scale by 1.
     */
    void incrCurrIntegrCount()
    {
        curr_integr_count_++;
    }

    /**
     * \brief Increment the number of observed voxels in at the current scale by 1.
     *
     * \param[in] do_increment The optional flag indicating if the counter should be incremented.
     */
    void incrCurrObservedCount(bool do_increment = true);

    /**
     * \brief Reset the current integration and observation count to 0.
     */
    void resetCurrCount();

    /**
     * \brief When a block is initialised from an observed block (i.e. init_data.observed == true), set the current
     *        observed count to all voxels observed and the integration count to the nodes value. Otherwise reset the current
     *        count.
     */
    void initCurrCout();

    /**
     * \return The integration scale of the buffer.
     */
    int buffer_scale() const
    {
        return buffer_scale_;
    }
    size_t bufferIntegrCount() const
    {
        return buffer_integr_count_;
    }
    size_t bufferObservedCount() const
    {
        return buffer_observed_count_;
    }

    /**
     * \brief Increment the buffer count if incrementation criterion is met.
     *        I.e. the scale normalised number of observations at the buffer scale >= 95% observations at the current scale.
     */
    void incrBufferIntegrCount(const bool do_increment = true);


    /**
     * \brief Increment the number of observed voxels at the buffers scale by 1.
     *
     * \param[in] do_increment The optional flag indicating if the counter should be incremented.
     */
    void incrBufferObservedCount(const bool do_increment = true);
    /**
     * \brief Reset the buffer integration and observation count to 0.
     */
    void resetBufferCount();

    /**
     *  \brief Reset buffer variables to the initial values and free the buffer data if applicable.
     */
    void resetBuffer();

    /**
     * \brief Init buffer variables.
     *
     * \param[in] buffer_scale The scale the buffer should be initialised at.
     */
    void initBuffer(const int buffer_scale);

    /**
     * \brief Check if the scale should be switched from the current scale to the recommended.
     *
     * \return True is data is switched to recommended scale.
     */
    bool switchData();

    /**
     * \brief Get a `const` reference to the voxel data in the buffer at the voxel coordinates.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_coord The voxel coordinates of the data to be accessed.
     *
     * \return `const` reference to the voxel data in the buffer for the provided voxel coordinates.
     */
    const DataType& bufferData(const Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Get a reference to the voxel data in the buffer at the voxel coordinates.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_coord The voxel coordinates of the data to be accessed.
     *
     * \return Reference to the voxel data in the buffer for the provided voxel coordinates.
     */
    DataType& bufferData(const Eigen::Vector3i& voxel_coord);

    /**
     * \brief Get a `const` reference to the voxel data in the buffer at the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return `const` reference to the voxel data in the buffer for the provided voxel index.
     */
    const DataType& bufferData(const int voxel_idx) const
    {
        assert(buffer_data_);
        assert(voxel_idx >= 0);
        assert(voxel_idx < math::cu(BlockSize >> buffer_scale_));
        return buffer_data_[voxel_idx];
    }

    /**
     * \brief Get a reference to the voxel data in the buffer at the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return Reference to the voxel data in the buffer for the provided voxel index.
     */
    DataType& bufferData(const int voxel_idx)
    {
        assert(buffer_data_);
        assert(voxel_idx >= 0);
        assert(voxel_idx < math::cu(BlockSize >> buffer_scale_));
        return buffer_data_[voxel_idx];
    }

    /**
     * \brief Get a pointer to the mean block data array at a given scale.
     *
     * \param[in] scale The scale to return the mean block data array from.
     *
     * \return The pointer to the mean block data array at the provided scale.
     *         Returns a nullptr if the scale smaller than the min allocated scale.
     */
    DataType* blockDataAtScale(const int scale);

    /**
     * \brief Get a pointer to the min block data array at a given scale.
     *
     * \param[in] scale The scale to return the min block data array from.
     *
     * \return The pointer to the min block data array at the provided scale.
     *         Returns a nullptr if the scale smaller than the min allocated scale.
     */
    DataType* blockMinDataAtScale(const int scale);
    /**
     * \brief Get a pointer to the max block data array at a given scale.
     *
     * \param[in] scale The scale to return the max block data array from.
     *
     * \return The pointer to the max block data array at the provided scale.
     *         Returns a nullptr if the scale smaller than the min allocated scale.
     */
    DataType* blockMaxDataAtScale(const int scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    std::vector<DataType*> block_data_;
    std::vector<DataType*> block_min_data_;
    std::vector<DataType*> block_max_data_;

    size_t curr_integr_count_;      ///<< Number of integrations at that current scale.
    size_t curr_observed_count_;    ///<< Number of observed voxels at the current scale

    /**
     * \brief Rather than switching directly to a different integration scale once the integration scale computation
     *        recommends a different scale, data is continued to be integrated at the current scale and additionally into
     *        a buffer at the recommended scale.
     *        Recommended scale == current scale:
     *            The buffer_data_ points to a `nullptr`
     *        Recommended scale < current scale:
     *            The buffer_data_ points to a independently allocated array of voxel data. The data is initialised with
     *            the parent data at the current integration scale. Once the scale changes the data is inserted into the
     *            block_data_ and block_max_data_ vector.
     *        Recommended scale > current scale:
     *            The buffer_data_ points to according scale in the block_data_ vector. The data integration starts from
     *            the mean up-propagated value. Up until the recommened scale > current scale the mean up-propagation starts
     *            from the recommened scale such that the data is not overwritten by the up-propagation from the current scale.
     *            However the max up-propagation continues from the current integration scale. Once the scale changes the
     *            current_data_ and current_scale_ is set to the buffer setup, the finest scale in the block_data_ and
     *            block_max_data_ is deleted and the new finest scales in the buffers adjusted accordingly.
     *
     * \note  The recommended scale can only differ by +/-1 scale from the current scale.
     *        The overhead of integrating at two different scales is insignificant compared to switching immediately as
     *        the double integration only happens in areas where the recommended integration scale changed and stops
     *        as soon as the criteria for switching to the finer or coarser scale.
     */
    DataType* buffer_data_ = nullptr; ///<< Pointer to the buffer data.
    int buffer_scale_ = -1;           ///<< The scale of the buffer.
    size_t
        buffer_integr_count_; ///<< Number of integrations at the buffer scale. \note Is only incremented when 95% of the current observations are reached.
    size_t buffer_observed_count_; ///<< Number of observed voxels in the buffer.

    const DerivedT* underlying() const
    {
        return static_cast<const DerivedT*>(this);
    }
};



/** A leaf node of an se::Octree.
 *
 * \tparam DataT     The type of data stored in the octree.
 * \tparam ResT      The value of se::Res for the octree.
 * \tparam BlockSize The edge length of se::Block in voxels.
 */
template<typename DataT, Res ResT, int BlockSize>
class Block : public OctantBase,
              public std::conditional<
                  ResT == Res::Single,
                  BlockData<DataT, ResT, BlockSize>,
                  BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::type

{
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
