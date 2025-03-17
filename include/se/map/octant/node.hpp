/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NODE_HPP
#define SE_NODE_HPP

namespace se {

/** Contains se::Data stored in se::Node and appropriate methods. Partial template specilization is
 * used so that se::Node doesn't contain unnecessary data. This non-specialized version contains no
 * data.
 */
template<typename DataT, typename DerivedT>
struct NodeData {
    protected:
    NodeData(const DataT&)
    {
    }

    public:
    /** Always returns the default data. This method allows simplifying the implementation of
     * certain algorithms.
     */
    const DataT& getData() const
    {
        static const DataT default_data;
        return default_data;
    }
};



/** Specialization of se::NodeData for se::Field::Occupancy. It contains minimum and maximum
 * up-propagated data.
 */
template<Colour ColB, Semantics SemB, typename DerivedT>
struct NodeData<Data<Field::Occupancy, ColB, SemB>, DerivedT> {
    typedef Data<Field::Occupancy, ColB, SemB> DataType;

    protected:
    /** Construct a node with both its minimum and maximum data initialized to \p init_data. */
    NodeData(const DataType& init_data)
    {
        min_data_ = init_data;
        max_data_ = init_data;
    }

    public:
    /** Return the node data. If the node is not observed and not a leaf the default data is
     * returned.
     */
    const DataType& getData() const
    {
        static const DataType default_data = DataType();
        return (max_data_.field.observed && underlying()->isLeaf()) ? getMaxData() : default_data;
    }

    /** Return the minimum data among the node's children. */
    const DataType& getMinData() const
    {
        return min_data_;
    }

    /** Return the maximum data among the node's children. */
    const DataType& getMaxData() const
    {
        return max_data_;
    }

    /** Set the node's data to \p data. It should only be called on leaf nodes. */
    void setData(const DataType& data)
    {
        assert(underlying()->isLeaf());
        setMinData(data);
        setMaxData(data);
    }

    /** Set the minimum data among the node's children to \p data. */
    void setMinData(const DataType& data)
    {
        min_data_ = data;
    }

    /** Set the maximum data among the node's children to \p data. */
    void setMaxData(const DataType& data)
    {
        max_data_ = data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /** The minimum data among the node's children or the node's data if it's a leaf. */
    DataType min_data_;
    /** The maximum data among the node's children or the node's data if it's a leaf. */
    DataType max_data_;

    const DerivedT* underlying() const
    {
        return static_cast<const DerivedT*>(this);
    }
};



/** An intermediate node of an se::Octree.
 *
 * An se::Node is never a leaf in TSDF octrees but may be a leaf in occupancy octrees.
 *
 * \tparam DataT The type of data stored in the octree.
 * \tparam ResT  The value of se::Res for the octree.
 */
template<typename DataT, Res ResT>
class Node : public OctantBase, public NodeData<DataT, Node<DataT, ResT>> {
    public:
    typedef DataT DataType;

    /** Construct a node at coordinates \p coord in voxels, with an edge length \p size in voxels
     * and initialize its data with \p init_data.
     *
     * \warning This constructor should only be used for the octree root node as it doesn't set the
     * parent pointer.
     */
    Node(const Eigen::Vector3i& coord, const int size, const DataT& init_data);

    /** Construct the child node of \p parent_ptr with index \p child_idx and initialize its data
     * with \p init_data. The value of \p child_idx must be in the interval [0, 7] inclusive.
     */
    Node(Node* const parent_ptr, const int child_idx, const DataT& init_data);

    /** Return the edge length of the node in voxels */
    int getSize() const;

    /** Return a pointer to the node child with index \p child_idx. The value of \p child_idx must
     * be in the interval [0, 7] inclusive. Returns nullptr if the child is not allocated.
     */
    const OctantBase* getChild(const int child_idx) const;

    /** A non-const overload of the previous member function. */
    OctantBase* getChild(const int child_idx);

    /** Set the node child with index \p child_idx to \p child_ptr. The value of \p child_idx must
     * be in the interval [0, 7] inclusive.
     */
    void setChild(const int child_idx, OctantBase* const child_ptr);

    /** Return the coordinates in voxels of the child with index \p child_idx. */
    Eigen::Vector3i getChildCoord(const int child_idx) const;

    /** Return the index of the child of the node with coordinates \p child_coord. The returned
     * index is in the interval [0, 7] inclusive.
     *
     * \warning Will return garbage if \p child_coord doesn't correspond to the coordinates of a
     * child of the node.
     */
    int getChildIdx(const Eigen::Vector3i& child_coord) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    // Pointers to the eight node children. Must be nullptr for unallocated children.
    std::array<OctantBase*, 8> children_ptr_;
    // The edge length of the node in voxels.
    const int size_;
};

} // namespace se

#include "impl/node_impl.hpp"

#endif // SE_NODE_HPP
