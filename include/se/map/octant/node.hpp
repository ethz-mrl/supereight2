/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NODE_HPP
#define SE_NODE_HPP

namespace se {

template<typename DataT, Res ResT>
class Node;

/** Contains se::Data stored in se::Node and appropriate methods. Partial template specilization is
 * used so that se::Node doesn't contain unnecessary data. This non-specialized version contains no
 * data.
 */
template<typename DataT, Res ResT>
struct NodeData {
    /** Always returns the default data. This method allows simplifying the implementation of
     * certain algorithms.
     */
    const DataT& data() const
    {
        static const DataT default_data;
        return default_data;
    }

    protected:
    NodeData(const DataT&)
    {
    }
};



/** Specialization of se::NodeData for se::Field::Occupancy. It contains minimum and maximum
 * up-propagated data.
 */
template<Colour ColB, Id IdB>
struct NodeData<Data<Field::Occupancy, ColB, IdB>, Res::Multi> {
    typedef Data<Field::Occupancy, ColB, IdB> DataType;

    /** The minimum data among the node's children or the node's data if it's a leaf. */
    DataType min_data;
    /** The maximum data among the node's children or the node's data if it's a leaf. */
    DataType max_data;

    /** Return NodeData::max_data if the node is observed and a leaf, the default data otherwise. */
    const DataType& data() const
    {
        static const DataType default_data = DataType();
        return (max_data.field.observed && derived()->isLeaf()) ? max_data : default_data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    /** Construct a node with both its minimum and maximum data initialized to \p init_data. */
    NodeData(const DataType& init_data)
    {
        min_data = init_data;
        max_data = init_data;
    }

    private:
    /** Cast to the derived class se::Node to allow accessing its members. */
    const Node<DataType, Res::Multi>* derived() const
    {
        return static_cast<const Node<DataType, Res::Multi>*>(this);
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
class Node : public OctantBase, public NodeData<DataT, ResT> {
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

    /** Return a pointer to the node child with index \p child_idx. The value of \p child_idx must
     * be in the interval [0, 7] inclusive. Returns nullptr if the child is not allocated.
     */
    OctantBase* getChild(const int child_idx);

    /** \constoverload */
    const OctantBase* getChild(const int child_idx) const;

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

    public:
    /** The edge length of the node in voxels. */
    const int size;
};

} // namespace se

#include "impl/node_impl.hpp"

#endif // SE_NODE_HPP
