/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SINGLERES_TSDF_UPDATER_HPP
#define SE_SINGLERES_TSDF_UPDATER_HPP

#include <se/map/octant/octant.hpp>

namespace se {

/** Specialization of se::Updater for single-resolution TSDF mapping. */
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
struct Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT> {
    typedef Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::BlockType BlockType;

    Updater(MapType& map,
            std::vector<OctantBase*>& block_ptrs,
            const timestamp_t timestamp,
            const Measurements<SensorT>& measurements);
};

} // namespace se

#include "impl/singleres_tsdf_updater_impl.hpp"

#endif // SE_SINGLERES_TSDF_UPDATER_HPP
