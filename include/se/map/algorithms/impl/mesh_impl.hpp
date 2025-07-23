/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_IMPL_HPP
#define SE_MESH_IMPL_HPP

namespace se {

template<Colour ColB, Id IdB>
TriangleMesh<ColB, IdB> quad_to_triangle_mesh(const QuadMesh<ColB, IdB>& quad_mesh)
{
    // Contains the indices of the quad vertices that should be used for the vertices of each of the
    // two resulting triangles.
    static constexpr std::array<std::array<int, 3>, 2> tri_to_quad = {{{0, 1, 2}, {0, 2, 3}}};
    TriangleMesh<ColB, IdB> triangle_mesh;
    triangle_mesh.reserve(2 * quad_mesh.size());
    for (const auto& quad : quad_mesh) {
        for (const auto& indices : tri_to_quad) {
            auto& triangle = triangle_mesh.emplace_back();
            triangle.scale = quad.scale;
            if constexpr (ColB == Colour::On) {
                if (quad.colour.vertexes) {
                    triangle.colour.vertexes.emplace();
                }
                triangle.colour.face = quad.colour.face;
            }
            for (size_t i = 0; i < indices.size(); i++) {
                triangle.vertexes[i] = quad.vertexes[indices[i]];
                if constexpr (ColB == Colour::On) {
                    if (quad.colour.vertexes) {
                        (*triangle.colour.vertexes)[i] = (*quad.colour.vertexes)[indices[i]];
                    }
                    triangle.colour.face = quad.colour.face;
                }
                if constexpr (IdB == Id::On) {
                    triangle.id.id = quad.id.id;
                }
            }
        }
    }
    return triangle_mesh;
}



namespace id {

template<typename FaceT, typename>
std::map<id_t, IdInfo> mesh_id_info(const Mesh<FaceT>& mesh)
{
    std::map<id_t, IdInfo> info;
    for (const auto& face : mesh) {
        const auto id = face.id.id;
        if (id) {
            const auto [it, _] = info.try_emplace(id);
            for (size_t v = 0; v < FaceT::num_vertexes; v++) {
                const auto& vertex = face.vertexes[v];
                it->second.centroid += vertex;
                it->second.aabb.extend(vertex);
                it->second.num_vertices++;
            }
        }
    }
    for (auto& p : info) {
        p.second.centroid /= p.second.num_vertices;
    }
    return info;
}



template<typename FaceT, typename>
std::map<id_t, Mesh<FaceT>> extract_id_meshes(const Mesh<FaceT>& mesh)
{
    return extract_id_meshes(mesh, [](const id_t id) { return id > 0; });
}



template<typename FaceT, typename ExtractIdF, typename>
std::map<id_t, Mesh<FaceT>> extract_id_meshes(const Mesh<FaceT>& mesh, ExtractIdF extract_id)
{
    std::map<id_t, Mesh<FaceT>> meshes;
    for (const auto& face : mesh) {
        const auto id = face.id.id;
        if (extract_id(id)) {
            const auto [it, _] = meshes.try_emplace(id);
            it->second.push_back(face);
        }
    }
    return meshes;
}



template<typename FaceT, typename>
void colour_mesh_by_id(Mesh<FaceT>& mesh,
                       const bool enable_shading,
                       const Eigen::Vector3f& light_dir_W,
                       const RGB ambient_light)
{
    const Eigen::Vector3f ambient_light_f(ambient_light.r, ambient_light.g, ambient_light.b);
    for (auto& face : mesh) {
        face.colour.face = id_colour(face.id.id);
        if (enable_shading) {
            const Eigen::Vector3f diffuse_colour(
                (*face.colour.face).r, (*face.colour.face).g, (*face.colour.face).b);
            const Eigen::Vector3f surface_normal_W =
                math::plane_normal(face.vertexes[0], face.vertexes[1], face.vertexes[2]);
            const float intensity = std::max(surface_normal_W.dot(light_dir_W), 0.0f);
            Eigen::Vector3f col = intensity * diffuse_colour + ambient_light_f;
            se::eigen::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Constant(255.0f));
            (*face.colour.face).r = col.x();
            (*face.colour.face).g = col.y();
            (*face.colour.face).b = col.z();
        }
    }
}

} // namespace id



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
