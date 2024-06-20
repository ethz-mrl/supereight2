/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_IMPL_HPP
#define SE_MESH_IMPL_HPP

namespace se {

template<Colour ColB, Semantics SemB>
TriangleMesh<ColB, SemB> quad_to_triangle_mesh(const QuadMesh<ColB, SemB>& quad_mesh)
{
    // Contains the indices of the quad vertices that should be used for the vertices of each of the
    // two resulting triangles.
    static constexpr std::array<std::array<int, 3>, 2> tri_to_quad = {{{0, 1, 2}, {0, 2, 3}}};
    TriangleMesh<ColB, SemB> triangle_mesh;
    triangle_mesh.reserve(2 * quad_mesh.size());
    for (const auto& quad : quad_mesh) {
        for (const auto& indices : tri_to_quad) {
            auto& triangle = triangle_mesh.emplace_back();
            triangle.scale = quad.scale;
            for (size_t i = 0; i < indices.size(); i++) {
                triangle.vertexes[i] = quad.vertexes[indices[i]];
                if constexpr (ColB == Colour::On) {
                    triangle.colour.vertexes[i] = quad.colour.vertexes[indices[i]];
                }
                if constexpr (SemB == Semantics::On) {
                    triangle.semantic.segment_id = quad.semantic.segment_id;
                }
            }
        }
    }
    return triangle_mesh;
}

namespace meshing {

template<size_t NumFaceVertices>
void VertexIndexMesh<NumFaceVertices>::merge(const VertexIndexMesh<NumFaceVertices>& other)
{
    const size_t old_size = vertices.size();
    vertices.reserve(old_size + other.vertices.size());
    std::copy(other.vertices.begin(), other.vertices.end(), std::back_inserter(vertices));

    indices.reserve(indices.size() + other.indices.size());
    std::transform(other.indices.begin(),
                   other.indices.end(),
                   std::back_inserter(indices),
                   [old_size](auto index) { return index + old_size; });
}

template<size_t NumFaceVertices>
void VertexIndexMesh<NumFaceVertices>::compute_normals()
{
    static_assert(NumFaceVertices == 3 || NumFaceVertices == 4,
                  "Only triangle and quad meshes are supported");

    if (indices.size() % NumFaceVertices != 0) {
        throw std::runtime_error("Invalid number of indices");
    }

    for (size_t i = 0; i < indices.size(); i += NumFaceVertices) {
        // TODO: make the code below work for any number of NumFaceVertices.
        auto& v0 = vertices[indices[i]];
        auto& v1 = vertices[indices[i + 1]];
        auto& v2 = vertices[indices[i + 2]];

        auto normal = (v1.position - v0.position).cross(v2.position - v0.position).normalized();

        if constexpr (NumFaceVertices == 4) {
            auto& v3 = vertices[indices[i + 3]];
            normal += (v2.position - v0.position).cross(v3.position - v0.position).normalized();
        }

        v0.normal.has_value() ? v0.normal.value() += normal : v0.normal = normal;
        v1.normal.has_value() ? v1.normal.value() += normal : v1.normal = normal;
        v2.normal.has_value() ? v2.normal.value() += normal : v2.normal = normal;

        if constexpr (NumFaceVertices == 4) {
            auto& v3 = vertices[indices[i + 3]];
            v3.normal.has_value() ? v3.normal.value() += normal : v3.normal = normal;
        }
    }

    for (auto& vertex : vertices) {
        if (vertex.normal) {
            vertex.normal->normalize();
        }
    }
}

} // namespace meshing

} // namespace se

#endif // SE_MESH_IMPL_HPP
