/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_ID_IMPL_HPP
#define SE_DATA_ID_IMPL_HPP

namespace se {

inline bool IdData<Id::On>::update(const id_t id_)
{
    if (id_ != g_no_id) {
        id = id_;
        return true;
    }
    return false;
}

} // namespace se

#endif // SE_DATA_ID_IMPL_HPP
