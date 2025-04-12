/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/data.hpp"

#include <cmath>
#include <se/common/str_utils.hpp>
#include <se/common/yaml.hpp>

namespace se {

void FieldData<Field::Occupancy>::Config::readYaml(const std::string& yaml_file)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
    }

    // Get the node containing the data configuration.
    const cv::FileNode node = fs["data"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default data configuration, no \"data\" section found in "
                  << yaml_file << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_float(node, "k_sigma", k_sigma);
    se::yaml::subnode_as_float(node, "sigma_min_factor", sigma_min_factor);
    se::yaml::subnode_as_float(node, "sigma_max_factor", sigma_max_factor);
    se::yaml::subnode_as_float(node, "k_tau", k_tau);
    se::yaml::subnode_as_float(node, "tau_min_factor", tau_min_factor);
    se::yaml::subnode_as_float(node, "tau_max_factor", tau_max_factor);
    se::yaml::subnode_as_float(node, "log_odd_min", log_odd_min);
    se::yaml::subnode_as_float(node, "log_odd_max", log_odd_max);
    se::yaml::subnode_as_int(node, "fs_integr_scale", fs_integr_scale);
    std::string uncertainty_model_s;
    se::yaml::subnode_as_string(node, "uncertainty_model", uncertainty_model_s);

    if (uncertainty_model_s == "linear") {
        uncertainty_model = UncertaintyModel::Linear;
    }

    if (uncertainty_model_s == "quadratic") {
        uncertainty_model = UncertaintyModel::Quadratic;
    }

    max_weight =
        std::floor(std::fabs(FieldData<Field::Occupancy>::min_occupancy / (0.97f * log_odd_min)));
}



std::ostream& operator<<(std::ostream& os, const FieldData<Field::Occupancy>::Config& c)
{
    os << str_utils::value_to_pretty_str(c.k_sigma, "k_sigma") << "\n";
    os << str_utils::value_to_pretty_str(c.sigma_min_factor, "sigma_min_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.sigma_max_factor, "sigma_max_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.k_tau, "k_tau") << "\n";
    os << str_utils::value_to_pretty_str(c.tau_min_factor, "tau_min_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.tau_max_factor, "tau_max_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.log_odd_min, "log_odd_min") << "\n";
    os << str_utils::value_to_pretty_str(c.log_odd_max, "log_odd_max") << "\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    os << str_utils::value_to_pretty_str(c.fs_integr_scale, "fs_integr_scale") << "\n";
    os << str_utils::str_to_pretty_str(
        ((c.uncertainty_model == UncertaintyModel::Linear) ? "linear" : "quadratic"),
        "uncertainty_model")
       << "\n";
    return os;
}



void FieldData<Field::TSDF>::Config::readYaml(const std::string& yaml_file)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
    }

    // Get the node containing the data configuration.
    const cv::FileNode node = fs["data"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default data configuration, no \"data\" section found in "
                  << yaml_file << "\n";
        return;
    }

    se::yaml::subnode_as_float(node, "truncation_boundary_factor", truncation_boundary_factor);
    se::yaml::subnode_as_float(node, "max_weight", max_weight);
}



std::ostream& operator<<(std::ostream& os, const FieldData<Field::TSDF>::Config& c)
{
    os << str_utils::value_to_pretty_str(c.truncation_boundary_factor, "truncation_boundary_factor")
       << "x\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    return os;
}



void ColourData<se::Colour::On>::Config::readYaml(const std::string& /* yaml_file */)
{
}



std::ostream& operator<<(std::ostream& os, const ColourData<se::Colour::Off>::Config& /* c */)
{
    return os;
}

std::ostream& operator<<(std::ostream& os, const ColourData<se::Colour::On>::Config& /* c */)
{
    return os;
}



void IdData<se::Id::On>::Config::readYaml(const std::string& /* yaml_file */)
{
    // TODO Implement when Id fusion is added.
}



std::ostream& operator<<(std::ostream& os, const IdData<se::Id::Off>::Config& /* c */)
{
    return os;
}

std::ostream& operator<<(std::ostream& os, const IdData<se::Id::On>::Config& /* c */)
{
    return os;
}

} // namespace se
