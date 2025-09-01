/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ITERATOR_HPP
#define SE_ITERATOR_HPP

#include <se/map/utils/octant_util.hpp>
#include <stack>



namespace se {

template<typename DerivedT>
struct BaseTraits;

/** \brief Iterates over all valid data in the octree at the last scale it
 * was updated at.
 * The iterator performs a depth-first traversal of the octree. To use it
 * just use the se::Octree::begin() and se::Octree::end() functions or a
 * range-based for loop:
 *
 * ``` cpp
 * for (auto& volume : octree) {
 *     // Do stuff with volume
 * }
 * ```
 *
 * \note Changes to the se::Octree while iterating will result in strange
 * behavior.
 */
template<typename DerivedT>
struct BaseIterator {
    typedef typename BaseTraits<DerivedT>::OctreeType OctreeType;
    typedef typename OctreeType::NodeType NodeType;

    BaseIterator& operator++();

    BaseIterator operator++(int);

    bool operator==(const BaseIterator& other) const;

    bool operator!=(const BaseIterator& other) const;

    OctantBase* operator*() const;

    // Iterator traits
    using difference_type = long;
    using value_type = OctantBase;
    using pointer = OctantBase*;
    using reference = OctantBase&;
    using iterator_category = std::forward_iterator_tag;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    BaseIterator();

    BaseIterator(OctreeType* const octree_ptr);

    // Find the next Volume with valid data.
    void nextData();

    private:
    OctreeType* octree_ptr_ = nullptr;
    OctantBase* current_octant_ptr_ = nullptr;
    std::stack<OctantBase*> octant_stack_;
};



template<typename OctreeT>
struct OctreeIterator : public BaseIterator<OctreeIterator<OctreeT>> {
    OctreeIterator() : BaseIterator<OctreeIterator<OctreeT>>()
    {
    }

    OctreeIterator(OctreeT* octree_ptr) : BaseIterator<OctreeIterator<OctreeT>>(octree_ptr)
    {
        this->nextData();
    }

    bool isNext(OctantBase* /* octant_ptr */)
    {
        return true;
    }

    static constexpr bool has_ignore_condition = false;
};



template<typename OctreeT>
struct NodesIterator : public BaseIterator<NodesIterator<OctreeT>> {
    NodesIterator() : BaseIterator<NodesIterator<OctreeT>>()
    {
    }

    NodesIterator(OctreeT* octree_ptr) : BaseIterator<NodesIterator<OctreeT>>(octree_ptr)
    {
        this->nextData();
    }

    bool isNext(OctantBase* octant_ptr)
    {
        return !octant_ptr->is_block;
    }

    static constexpr bool has_ignore_condition = false;
};



template<typename OctreeT>
struct BlocksIterator : public BaseIterator<BlocksIterator<OctreeT>> {
    BlocksIterator() : BaseIterator<BlocksIterator<OctreeT>>()
    {
    }

    BlocksIterator(OctreeT* octree_ptr) : BaseIterator<BlocksIterator<OctreeT>>(octree_ptr)
    {
        this->nextData();
    }

    bool isNext(OctantBase* octant_ptr)
    {
        return octant_ptr->is_block;
    }

    static constexpr bool has_ignore_condition = false;
};



template<typename OctreeT>
struct LeavesIterator : public BaseIterator<LeavesIterator<OctreeT>> {
    LeavesIterator() : BaseIterator<LeavesIterator<OctreeT>>()
    {
    }

    LeavesIterator(OctreeT* octree_ptr) : BaseIterator<LeavesIterator<OctreeT>>(octree_ptr)
    {
        this->nextData();
    }

    bool isNext(OctantBase* octant_ptr)
    {
        return octant_ptr->isLeaf();
    }

    static constexpr bool has_ignore_condition = false;
};



template<typename OctreeT>
struct UpdateIterator : public BaseIterator<UpdateIterator<OctreeT>> {
    UpdateIterator() : BaseIterator<UpdateIterator<OctreeT>>(), time_stamp_(0)
    {
    }

    UpdateIterator(OctreeT* octree_ptr, timestamp_t time_stamp) :
            BaseIterator<UpdateIterator<OctreeT>>(octree_ptr), time_stamp_(time_stamp)
    {
        this->nextData();
    }

    bool isNext(OctantBase* octant_ptr)
    {
        return octant_ptr->is_block && octant_ptr->timestamp >= time_stamp_;
    }

    bool doIgnore(OctantBase* octant_ptr)
    {
        return octant_ptr->timestamp < time_stamp_;
    }

    const timestamp_t time_stamp_;

    static constexpr bool has_ignore_condition = true;
};


template<typename MapT, typename SensorT>
struct FrustumIterator : public BaseIterator<FrustumIterator<MapT, SensorT>> {
    typedef typename MapT::OctreeType OctreeType;

    FrustumIterator() : BaseIterator<FrustumIterator<MapT, SensorT>>()
    {
    }

    FrustumIterator(MapT& map, const SensorT& sensor, const Eigen::Isometry3f& T_SM) :
            BaseIterator<FrustumIterator<MapT, SensorT>>(&map.getOctree()),
            map_ptr_(&map),
            sensor_ptr_(&sensor),
            T_SM_(T_SM)
    {
        this->nextData();
    }

    bool isNext(OctantBase* octant_ptr)
    {
        return octant_ptr->is_block;
    }

    bool doIgnore(OctantBase* octant_ptr)
    {
        Eigen::Vector3f octant_centre_point_M;
        map_ptr_->voxelToPoint(octant_ptr->coord, octant_ptr->size, octant_centre_point_M);
        // Convert it to the sensor frame.
        const Eigen::Vector3f octant_centre_point_S = T_SM_ * octant_centre_point_M;

        float octant_radius = std::sqrt(3.0f) / 2.0f * map_ptr_->getRes() * octant_ptr->size;
        bool do_ignore = !sensor_ptr_->sphereInFrustum(octant_centre_point_S, octant_radius);
        return do_ignore;
    }

    static constexpr bool has_ignore_condition = true;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    MapT* map_ptr_;
    const SensorT* sensor_ptr_;
    const Eigen::Isometry3f T_SM_; // TODO: Needs to be ref?
};


// Declare and define a base_traits specialization for the octree iterator:
template<typename OctreeT>
struct BaseTraits<OctreeIterator<OctreeT>> {
    typedef OctreeT OctreeType;
};



// Declare and define a base_traits specialization for the nodes iterator:
template<typename OctreeT>
struct BaseTraits<NodesIterator<OctreeT>> {
    typedef OctreeT OctreeType;
};



// Declare and define a base_traits specialization for the block iterator:
template<typename OctreeT>
struct BaseTraits<BlocksIterator<OctreeT>> {
    typedef OctreeT OctreeType;
};



// Declare and define a base_traits specialization for the leaves iterator:
template<typename OctreeT>
struct BaseTraits<LeavesIterator<OctreeT>> {
    typedef OctreeT OctreeType;
};



// Declare and define a base_traits specialization for the update iterator:
template<typename OctreeT>
struct BaseTraits<UpdateIterator<OctreeT>> {
    typedef OctreeT OctreeType;
};

// Declare and define a base_traits specialization for the update iterator:
template<typename MapT, typename SensorT>
struct BaseTraits<FrustumIterator<MapT, SensorT>> {
    typedef typename MapT::OctreeType OctreeType;
};


} // namespace se

#include "impl/iterator_impl.hpp"

#endif // SE_ITERATOR_HPP
