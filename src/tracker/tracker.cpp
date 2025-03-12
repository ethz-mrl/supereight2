/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "se/tracker/tracker.hpp"

namespace se {

void TrackerConfig::readYaml(const std::string& filename)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << filename << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << filename << "\n";
        return;
    }

    // Get the node containing the tracker configuration.
    const cv::FileNode node = fs["tracker"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr
            << "Warning: using default tracker configuration, no \"tracker\" section found in "
            << filename << "\n";
        return;
    }

    // Read the config parameters.
    yaml::subnode_as_vector(node, "iterations", iterations);
    yaml::subnode_as_float(node, "dist_threshold", dist_threshold);
    yaml::subnode_as_float(node, "normal_threshold", normal_threshold);
    yaml::subnode_as_float(node, "track_threshold", track_threshold);
    yaml::subnode_as_float(node, "icp_threshold", icp_threshold);
}



std::ostream& operator<<(std::ostream& os, const TrackerConfig& c)
{
    os << str_utils::vector_to_pretty_str(c.iterations, "iterations");
    os << str_utils::value_to_pretty_str(c.dist_threshold, "dist_threshold") << "\n";
    os << str_utils::value_to_pretty_str(c.normal_threshold, "normal_threshold") << "\n";
    os << str_utils::value_to_pretty_str(c.track_threshold, "track_threshold") << "\n";
    os << str_utils::value_to_pretty_str(c.icp_threshold, "icp_threshold") << "\n";
    return os;
}
} // namespace se
