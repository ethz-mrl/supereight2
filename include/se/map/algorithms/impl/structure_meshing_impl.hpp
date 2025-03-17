/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_STRUCTURE_MESHING_IMPL_HPP
#define SE_STRUCTURE_MESHING_IMPL_HPP

namespace se {

template<typename OctreeT>
typename OctreeT::StructureMesh octree_structure_mesh(OctreeT& octree, const bool only_leaves)
{
    typedef typename OctreeT::StructureMesh::value_type Face;

    typename OctreeT::StructureMesh mesh;
    for (auto octant_it = octree.begin(); octant_it != octree.end(); ++octant_it) {
        const auto octant_ptr = *octant_it;
        if (only_leaves && !octant_ptr->isLeaf()) {
            continue;
        }
        int node_size;
        int node_scale;
        if (octant_ptr->is_block) {
            node_size = static_cast<typename OctreeT::BlockType*>(octant_ptr)->size;
            node_scale = static_cast<typename OctreeT::BlockType*>(octant_ptr)->current_scale;
        }
        else {
            node_size = static_cast<typename OctreeT::NodeType*>(octant_ptr)->size;
            // Since we don't care about the node scale, just set it to a number that will result in
            // a gray color when saving the mesh.
            node_scale = 7;
        }

        // Get the coordinates of the octant vertices.
        Eigen::Vector3f node_corners[8];
        const Eigen::Vector3i node_coord = octant_ptr->coord;
        node_corners[0] = node_coord.cast<float>();
        node_corners[1] = (node_coord + Eigen::Vector3i(node_size, 0, 0)).cast<float>();
        node_corners[2] = (node_coord + Eigen::Vector3i(0, node_size, 0)).cast<float>();
        node_corners[3] = (node_coord + Eigen::Vector3i(node_size, node_size, 0)).cast<float>();
        node_corners[4] = (node_coord + Eigen::Vector3i(0, 0, node_size)).cast<float>();
        node_corners[5] = (node_coord + Eigen::Vector3i(node_size, 0, node_size)).cast<float>();
        node_corners[6] = (node_coord + Eigen::Vector3i(0, node_size, node_size)).cast<float>();
        node_corners[7] =
            (node_coord + Eigen::Vector3i(node_size, node_size, node_size)).cast<float>();

        // The Face::num_vertexes vertex indices to node_corners for each of the 6 faces.
        int face_vertex_idx[6][Face::num_vertexes] = {
            {0, 2, 3, 1}, {1, 3, 7, 5}, {5, 7, 6, 4}, {0, 4, 6, 2}, {0, 1, 5, 4}, {2, 6, 7, 3}};

        // Create the octant faces.
        for (int f = 0; f < 6; ++f) {
            mesh.emplace_back();
            for (size_t v = 0; v < Face::num_vertexes; ++v) {
                mesh.back().vertexes[v] = node_corners[face_vertex_idx[f][v]];
                mesh.back().scale = node_scale;
            }
        }
    }
    return mesh;
}

} // namespace se

#endif // SE_STRUCTURE_MESHING_IMPL_HPP
