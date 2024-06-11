/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_INTEGRATOR_MEASUREMENT_HPP
#define SE_INTEGRATOR_MEASUREMENT_HPP

#include <Eigen/Geometry>
#include <optional>
#include <se/image/image.hpp>
#include <se/map/utils/type_util.hpp>

namespace se {

/** Contains an \p image captured from \p sensor at pose \p T_WC. */
template<typename SensorT, typename T>
struct Measurement {
    const Image<T>& image;
    const SensorT& sensor;
    const Eigen::Isometry3f T_WC;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Deduction guide to allow template argument deduction from aggregate initialization.
// https://en.cppreference.com/w/cpp/language/class_template_argument_deduction#User-defined_deduction_guides
template<typename SensorT, typename T>
Measurement(const Image<T>&, const SensorT&, const Eigen::Isometry3f&) -> Measurement<SensorT, T>;



/** Contains measurements from different modalities that must be integrated together. All
 * measurements must be from the same type of sensor, for example se::PinholeCamera, but can have
 * different sensor parameters.
 */
template<typename SensorT>
struct Measurements {
    Measurement<SensorT, float> depth;
    std::optional<Measurement<SensorT, colour_t>> colour = std::nullopt;
    std::optional<Measurement<SensorT, segment_id_t>> segments = std::nullopt;
    se::Image<float>* depth_sigma = nullptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Deduction guides to allow template argument deduction from aggregate initialization. They also
// allow aggregate initialization of std::optional<se::Measurement> members from se::Measurement.
// https://en.cppreference.com/w/cpp/language/class_template_argument_deduction#User-defined_deduction_guides
template<typename SensorT>
Measurements(const Measurement<SensorT, float>&) -> Measurements<SensorT>;

template<typename SensorT>
Measurements(const Measurement<SensorT, float>&, const Measurement<SensorT, colour_t>&)
    -> Measurements<SensorT>;

template<typename SensorT>
Measurements(const Measurement<SensorT, float>&,
             const Measurement<SensorT, colour_t>&,
             const Measurement<SensorT, segment_id_t>&) -> Measurements<SensorT>;

template<typename SensorT>
Measurements(const Measurement<SensorT, float>&,
             const Measurement<SensorT, colour_t>&,
             const Measurement<SensorT, segment_id_t>&,
             const Image<float>*) -> Measurements<SensorT>;

} // namespace se

#endif // SE_INTEGRATOR_MEASUREMENT_HPP
