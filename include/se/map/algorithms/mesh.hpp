/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_FACE_HPP
#define SE_MESH_FACE_HPP

#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <se/common/eigen_utils.hpp>
#include <se/common/math_util.hpp>
#include <se/common/rgb.hpp>
#include <se/map/utils/setup_util.hpp>
#include <se/map/utils/type_util.hpp>
#include <vector>

namespace se {

template<size_t NumVertexes, Colour ColB>
struct MeshFaceColourData {};

template<size_t NumVertexes>
struct MeshFaceColourData<NumVertexes, Colour::On> {
    std::array<RGB, NumVertexes> vertexes;
    RGB face;
};



template<size_t NumVertexes, Semantics SemB>
struct MeshFaceSemanticData {};

template<size_t NumVertexes>
struct MeshFaceSemanticData<NumVertexes, Semantics::On> {
    segment_id_t segment_id = segment_id_t(0);
};



template<size_t NumVertexes, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct MeshFace {
    std::array<Eigen::Vector3f, NumVertexes> vertexes;
    MeshFaceColourData<NumVertexes, ColB> colour;
    MeshFaceSemanticData<NumVertexes, SemB> semantic;
    std::int8_t scale = 0;

    static constexpr size_t num_vertexes = NumVertexes;
    static constexpr Colour col = ColB;
    static constexpr Semantics sem = SemB;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/** \brief Meshes are represented as lists of faces.
 *
 * \bug This representation has the inherent problem that there is vertex duplication. A more
 * advanced representation would be needed to alleviate this, e.g. a list of vertices and a list of
 * faces with indices to the list of faces.
 */
template<typename FaceT>
using Mesh = std::vector<FaceT>;

template<Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
using Triangle = MeshFace<3, ColB, SemB>;

template<Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
using TriangleMesh = Mesh<Triangle<ColB, SemB>>;

template<Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
using Quad = MeshFace<4, ColB, SemB>;

template<Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
using QuadMesh = Mesh<Quad<ColB, SemB>>;



/** Return a triangle mesh containig two triangles for each face of \p quad_mesh. */
template<Colour ColB, Semantics SemB>
TriangleMesh<ColB, SemB> quad_to_triangle_mesh(const QuadMesh<ColB, SemB>& quad_mesh);



namespace segment {

struct SegmentInfo {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::AlignedBox3f aabb;
    size_t num_vertices = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Return information about all segments in \p mesh. The spatial information is in the same
 * coordinate frame as \p mesh.
 */
template<typename FaceT, typename = std::enable_if_t<FaceT::sem == Semantics::On>>
std::map<segment_id_t, SegmentInfo> mesh_segment_info(const Mesh<FaceT>& mesh);

/** Extract per-segment meshes for all segments in \p mesh. */
template<typename FaceT, typename = std::enable_if_t<FaceT::sem == Semantics::On>>
std::map<segment_id_t, Mesh<FaceT>> extract_segment_meshes(const Mesh<FaceT>& mesh);

/** Extract per-segment meshes for all segments in \p mesh for whose IDs \p extract_segment returns
 * true. \p extract_segment must be a functor with the following signature:
 * \code{.cpp}
 * bool extract_segment(const segment_id_t);
 * \endcode
 */
template<typename FaceT,
         typename ExtractSegmentF,
         typename = std::enable_if_t<FaceT::sem == Semantics::On>>
std::map<segment_id_t, Mesh<FaceT>> extract_segment_meshes(const Mesh<FaceT>& mesh,
                                                           ExtractSegmentF extract_segment);

/** Colour the faces of \p mesh by their segment ID. */
template<typename FaceT, typename = std::enable_if_t<FaceT::sem == Semantics::On>>
void colour_mesh_by_segment(Mesh<FaceT>& mesh,
                            const bool enable_shading = true,
                            const Eigen::Vector3f& light_dir_W = Eigen::Vector3f(-1, 0, -1),
                            const RGB ambient = RGB{0x40, 0x40, 0x40});

} // namespace segment



namespace meshing {

struct Vertex {
    Vertex(const Eigen::Vector3f& position) : position(position)
    {
    }

    Eigen::Vector3f position;
    std::optional<Eigen::Vector3f> normal;
    std::optional<RGB> color;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<size_t NumFaceVertices = 3>
class VertexIndexMesh {
    public:
    std::vector<Vertex, Eigen::aligned_allocator<Vertex>> vertices;
    std::vector<size_t> indices; // faces

    static constexpr size_t num_face_vertices = NumFaceVertices;

    void merge(const VertexIndexMesh& other);

    void compute_normals();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace meshing
} // namespace se

#include "impl/mesh_impl.hpp"

#endif // SE_MESH_FACE_HPP
