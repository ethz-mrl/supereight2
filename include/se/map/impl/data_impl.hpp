/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_IMPL_HPP
#define SE_DATA_IMPL_HPP

namespace se {
namespace data {

template<Colour ColB, Id IdB>
int up_prop_mean(Data<Field::TSDF, ColB, IdB>& parent_data,
                 const std::array<Data<Field::TSDF, ColB, IdB>, 8>& child_data)
{
    static_assert(std::is_floating_point_v<field_t> && std::is_floating_point_v<weight_t>,
                  "The code must be modified to avoid overflows for non-floating-point types.");
    // Accumulate TSDF values and weights while keeping track of valid child indices.
    field_t tsdf_sum(0);
    weight_t weight_sum(0);
    BoundedVector<int, 8> valid_child_indices;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        const auto& d = child_data[child_idx];
        if (d.field.valid()) {
            valid_child_indices.push_back(child_idx);
            tsdf_sum += d.field.tsdf;
            weight_sum += d.field.weight;
        }
    }
    if (!valid_child_indices.empty()) {
        // Set the parent data to the mean of the valid child data.
        parent_data.field.tsdf = tsdf_sum / valid_child_indices.size();
        parent_data.field.weight = std::ceil(weight_sum / valid_child_indices.size());

        if constexpr (ColB == Colour::On) {
            // Perform mean colour up-propagation.
            BoundedVector<ColourData<Colour::On>, 8> valid_child_colour_data;
            for (const int child_idx : valid_child_indices) {
                valid_child_colour_data.push_back(child_data[child_idx].colour);
            }
            parent_data.colour.setToMean(valid_child_colour_data);
        }

        if constexpr (IdB == Id::On) {
            // TODO: aggregate the Id of the valid children (mean?).
        }
    }
    return valid_child_indices.size();
}



template<Colour ColB, Id IdB>
int up_prop_mean(Data<Field::Occupancy, ColB, IdB>& parent_data,
                 const std::array<Data<Field::Occupancy, ColB, IdB>, 8>& child_data)
{
    static_assert(std::is_floating_point_v<field_t> && std::is_floating_point_v<weight_t>,
                  "The code must be modified to avoid overflows for non-floating-point types.");
    // Accumulate occupancy values and weights while keeping track of valid child indices.
    field_t occupancy_sum(0);
    weight_t weight_sum(0);
    BoundedVector<int, 8> valid_child_indices;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        const auto& d = child_data[child_idx];
        if (d.field.valid()) {
            valid_child_indices.push_back(child_idx);
            occupancy_sum += d.field.occupancy;
            weight_sum += d.field.weight;
        }
    }
    if (!valid_child_indices.empty()) {
        parent_data.field.occupancy = occupancy_sum / valid_child_indices.size();
        parent_data.field.weight = std::ceil(weight_sum / valid_child_indices.size());
        // Even if all children are observed, we haven't observed the data at this scale.
        parent_data.field.observed = false;

        if constexpr (ColB == Colour::On) {
            // Perform mean colour up-propagation.
            BoundedVector<ColourData<Colour::On>, 8> valid_child_colour_data;
            for (const int child_idx : valid_child_indices) {
                valid_child_colour_data.push_back(child_data[child_idx].colour);
            }
            parent_data.colour.setToMean(valid_child_colour_data);
        }

        if constexpr (IdB == Id::On) {
            // TODO: aggregate the Id of the valid children (mean?).
        }
    }
    return static_cast<int>(valid_child_indices.size());
}



namespace detail {

template<Colour ColB, Id IdB, typename IsBetterF>
int up_prop_best(Data<Field::Occupancy, ColB, IdB>& parent_data,
                 const std::array<Data<Field::Occupancy, ColB, IdB>, 8>& child_data,
                 const field_t worst_occupancy,
                 IsBetterF is_better)
{
    field_t best_occupancy = worst_occupancy;
    int best_index = 0;
    int num_valid = 0;
    int num_observed = 0;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        const auto& d = child_data[child_idx];
        if (d.field.valid()) {
            num_valid++;
            num_observed += d.field.observed;
            const field_t child_occupancy = get_field(d);
            if (is_better(child_occupancy, best_occupancy)) {
                best_occupancy = child_occupancy;
                best_index = child_idx;
            }
        }
    }
    if (num_valid > 0) {
        parent_data = child_data[best_index];
        // The parent is observed if all children are observed in minimum/maximum up-propagation.
        parent_data.field.observed = (num_observed == 8);
    }
    return num_valid;
}

} // namespace detail



template<Colour ColB, Id IdB>
int up_prop_min(Data<Field::Occupancy, ColB, IdB>& parent_min_data,
                const std::array<Data<Field::Occupancy, ColB, IdB>, 8>& child_min_data)
{
    return detail::up_prop_best(parent_min_data,
                                child_min_data,
                                std::numeric_limits<field_t>::max(),
                                [](const field_t a, const field_t b) { return a < b; });
}



template<Colour ColB, Id IdB>
int up_prop_max(Data<Field::Occupancy, ColB, IdB>& parent_max_data,
                const std::array<Data<Field::Occupancy, ColB, IdB>, 8>& child_max_data)
{
    return detail::up_prop_best(parent_max_data,
                                child_max_data,
                                std::numeric_limits<field_t>::lowest(),
                                [](const field_t a, const field_t b) { return a > b; });
}

} // namespace data
} // namespace se

#endif // SE_DATA_IMPL_HPP
