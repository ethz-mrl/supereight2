/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ITERATOR_IMPL_HPP
#define SE_ITERATOR_IMPL_HPP

namespace se {



template<typename DerivedT>
BaseIterator<DerivedT>::BaseIterator()
{
}



template<typename DerivedT>
BaseIterator<DerivedT>::BaseIterator(OctreeType* octree_ptr) : octree_ptr_(octree_ptr)
{
    if (octree_ptr_) {
        assert(octree_ptr_->getRoot());
        octant_stack_.push(octree_ptr_->getRoot());
    }
}



template<typename DerivedT>
BaseIterator<DerivedT>& BaseIterator<DerivedT>::operator++()
{
    nextData();
    return *this;
}



template<typename DerivedT>
BaseIterator<DerivedT> BaseIterator<DerivedT>::operator++(int)
{
    BaseIterator<DerivedT> previous_state(*this);
    nextData();
    return previous_state;
}



template<typename DerivedT>
bool BaseIterator<DerivedT>::operator==(const BaseIterator& other) const
{
    // The volume_ is obtained from the current place in the octree so checking
    // it would be redundant.
    return (octant_stack_ == other.octant_stack_
            && current_octant_ptr_ == other.current_octant_ptr_);
}



template<typename DerivedT>
bool BaseIterator<DerivedT>::operator!=(const BaseIterator& other) const
{
    return !(*this == other);
}



template<typename DerivedT>
OctantBase* BaseIterator<DerivedT>::operator*() const
{
    return current_octant_ptr_;
}



template<typename DerivedT>
void BaseIterator<DerivedT>::init()
{
    nextData();
}



template<typename DerivedT>
void BaseIterator<DerivedT>::nextData()
{
    while (!octant_stack_.empty()) {
        // Get the data from the top of the stacks
        OctantBase* octant = octant_stack_.top();
        // Pop the node since we'll be done with it after this call
        octant_stack_.pop();

        if constexpr (DerivedT::has_ignore_condition == true) {
            if (static_cast<DerivedT*>(this)->doIgnore(octant)) {
                continue;
            }
        }

        if (octant != nullptr && !octant->is_block) {
            // Non-leaf Node, push all children to the stack
            for (int child_idx = 0; child_idx < 8; child_idx++) {
                OctantBase* child_ptr = static_cast<NodeType*>(octant)->getChild(child_idx);
                if (child_ptr) {
                    octant_stack_.push(child_ptr);
                }
            }
            // Then continue until a leaf is found
        }

        if (static_cast<DerivedT*>(this)->isNext(octant)) {
            current_octant_ptr_ = octant;
            return;
        }
    }
    current_octant_ptr_ = nullptr;
}

} // namespace se

#endif // SE_ITERATOR_IMPL_HPP
