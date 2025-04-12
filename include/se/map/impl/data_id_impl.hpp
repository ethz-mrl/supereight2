/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_ID_IMPL_HPP
#define SE_DATA_ID_IMPL_HPP

namespace se {

inline bool IdData<Id::On>::update(const segment_id_t segment_id_)
{
    if (segment_id_ != g_not_segmented) {
        segment_id = segment_id_;
        return true;
    }
    return false;
}

} // namespace se

#endif // SE_DATA_ID_IMPL_HPP
