/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_FIELD_HPP
#define SE_DATA_FIELD_HPP

#include <se/map/utils/setup_util.hpp>
#include <se/map/utils/type_util.hpp>

namespace se {

enum class UncertaintyModel { Linear, Quadratic };

template<Field FieldT>
struct FieldData {
    struct Config {
    };
};

template<>
struct FieldData<Field::Occupancy> {
    field_t occupancy = 0;
    weight_t weight = 0;
    bool observed = false;

    /** The direction of surface normals in occupancy fields is opposite from the direction of the
     * field gradient. This is because the occupancy field is negative in free space and positive in
     * occupied space.
     */
    static constexpr bool normals_along_gradient = false;
    static constexpr field_t surface_boundary = 0;
    static constexpr field_t min_occupancy = -100;
    static constexpr field_t max_occupancy = 100;

    /** Return whether the field data has been updated at least once. */
    bool valid() const;

    /** Perform a weighted average log-odds occupancy update and set the data to observed, while
     * ensuring the weight doesn't exceed \p max_weight. Return whether the data was updated.
     */
    bool update(const field_t occupancy, const weight_t max_weight);

    struct Config {
        float k_sigma = 0.052f;
        float sigma_min_factor = 1.5f;
        float sigma_max_factor = 6.0f;

        float k_tau = 0.026f;
        float tau_min_factor = 6.0f;
        float tau_max_factor = 16.0f;

        field_t log_odd_min = -5.015;
        field_t log_odd_max = 5.015;

        weight_t max_weight = std::floor(std::fabs(min_occupancy / (0.97f * log_odd_min)));

        int fs_integr_scale = 1;

        UncertaintyModel uncertainty_model = UncertaintyModel::Linear;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };
};

std::ostream& operator<<(std::ostream& os, const FieldData<Field::Occupancy>::Config& c);

template<>
struct FieldData<Field::TSDF> {
    field_t tsdf = 1;
    weight_t weight = 0;

    /** The direction of surface normals in TSDF fields is the same as the direction of the field
     * gradient. This is because the TSDF field is positive outside the surface and negative inside
     * it.
     */
    static constexpr bool normals_along_gradient = true;
    static constexpr field_t surface_boundary = 0;

    /** Return whether the field data has been updated at least once. */
    bool valid() const;

    /** Perform a weighted average TSDF update by truncating the SDF value \p sdf within \p
     * truncation_boundary, while ensuring the weight doesn't exceed \p max_weight. Return whether
     * the data was updated. Data isn't updated if \p sdf is less than \p -truncation_boundary.
     */
    bool update(const field_t sdf, const field_t truncation_boundary, const weight_t max_weight);

    struct Config {
        field_t truncation_boundary_factor = 8;
        weight_t max_weight = 100;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };
};

std::ostream& operator<<(std::ostream& os, const FieldData<Field::TSDF>::Config& c);

} // namespace se

#include "impl/data_field_impl.hpp"

#endif // SE_DATA_FIELD_HPP
