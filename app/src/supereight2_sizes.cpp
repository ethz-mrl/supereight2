/*
 * SPDX-FileCopyrightText: 2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iostream>
#include <regex>
#include <se/supereight.hpp>

// This needs to be a macro so that the argument (the map type) can be converted to a string.
#define sizes(t)                                                                               \
    do {                                                                                       \
        const std::string name = std::regex_replace(#t, std::regex("(se::|<>)"), "");          \
        std::cout << name << "\t" << sizeof(t) << "\t" << sizeof(t::OctreeType) << "\t"        \
                  << sizeof(se::OctantBase) << "\t" << sizeof(t::OctreeType::NodeType) << "\t" \
                  << sizeof(t::OctreeType::BlockType) << "\t" << sizeof(t::DataType) << "\n";  \
    } while (0);

int main()
{
    std::cout << "Type\t"
                 "Map (B)\t"
                 "Octree (B)\t"
                 "Octant (B)\t"
                 "Node (B)\t"
                 "Block (B)\t"
                 "Data (B)\n";
    sizes(se::OccupancyMap<>);
    sizes(se::OccupancyColMap<>);
    sizes(se::OccupancySemMap<>);
    sizes(se::OccupancyColSemMap<>);
    sizes(se::TSDFMap<>);
    sizes(se::TSDFColMap<>);
    sizes(se::TSDFSemMap<>);
    sizes(se::TSDFColSemMap<>);
    sizes(se::TSDFMap<se::Res::Multi>);
    sizes(se::TSDFColMap<se::Res::Multi>);
    sizes(se::TSDFSemMap<se::Res::Multi>);
    sizes(se::TSDFColSemMap<se::Res::Multi>);
}
