// =====================================================================================================================

#pragma once
#ifndef LAYEREDMESH_H
#define LAYEREDMESH_H

// =====================================================================================================================

#include <tuple>

#include <pmp/surface_mesh.h>
#include <pmp/io/io.h>
#include <pmp_character/node_types/Character.h>

#include "pmp_ext/defs.h"
#include "volumetric_types.h"
#include "volumetric_base/state/filenames.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

// Type definitions
using Prism = std::tuple<pmp::Face, Layer>;
using Quad = std::tuple<pmp::Edge, Layer>;
using LayerVertex = std::tuple<pmp::Vertex, Layer>;

// =====================================================================================================================

// Utility functions
Layer getNextLayer(Layer inner);

inline Layer getLayer(int i) {
    if (i==0) return Layer::SKEL;
    if (i==1) return Layer::MUSC;
    if (i==2) return Layer::SKIN;
    throw pmp::InvalidInputException("Invalid layer index");
}

// ---------------------------------------------------------------------------------------------------------------------

// LayeredMesh class definition
class LayeredMesh : public pmp::SurfaceMesh {
public:
    // Constructors
    LayeredMesh() = default;

    LayeredMesh(const SurfaceMesh& skeleton, const SurfaceMesh& muscle, const SurfaceMesh& skin);

    // Initialization
    void initialize(const SurfaceMesh& skeleton, const SurfaceMesh& muscle, const SurfaceMesh& skin);

    void writePrism(const Prism & prism) const {
        auto [p1,p2,p3,p4,p5,p6] = getPositions(prism);
        std::cout << "np.array([" << std::endl;
        std::cout << "    [" << p1.x() << ", " << p1.y() << ", " << p1.z() << "]," << std::endl;
        std::cout << "    [" << p2.x() << ", " << p2.y() << ", " << p2.z() << "]," << std::endl;
        std::cout << "    [" << p3.x() << ", " << p3.y() << ", " << p3.z() << "]," << std::endl;
        std::cout << "    [" << p4.x() << ", " << p4.y() << ", " << p4.z() << "]," << std::endl;
        std::cout << "    [" << p5.x() << ", " << p5.y() << ", " << p5.z() << "]," << std::endl;
        std::cout << "    [" << p6.x() << ", " << p6.y() << ", " << p6.z() << "]" << std::endl;
        std::cout << "])" << std::endl;
    }

    void set_layer(const SurfaceMesh & s1, Layer layer) const {
        for (pmp::Vertex v: vertices()) {
            position(v,layer) = s1.position(v);
        }
    }

    [[nodiscard]]
        Eigen::MatrixXd toVertexNormalMatrix() const
    {
        Eigen::MatrixXd normals = Eigen::MatrixXd::Zero(static_cast<long>(3 * n_vertices()), 3);

        for (int l = 0; l < 3; ++l) {
            Layer layer = getLayer(l);

            // Accumulate face normals onto vertices
            for (auto f : faces()) {
                Eigen::Vector3d fn(0.0, 0.0, 0.0);
                auto vit = vertices(f);
                auto it = vit.begin();

                pmp::Vertex v0 = *it++;
                pmp::Vertex v1 = *it++;
                Eigen::Vector3d p0 = position(v0, layer);
                Eigen::Vector3d p1 = position(v1, layer);

                for (; it != vit.end(); ++it) {
                    pmp::Vertex v2 = *it;
                    Eigen::Vector3d p2 = position(v2, layer);
                    fn += (p1 - p0).cross(p2 - p0);
                    p1 = p2;
                }
                // Add this face normal to all vertices of the face
                for (auto v : vertices(f)) {
                    normals.row(Index(v, layer)) += fn.transpose();
                }
            }
        }

        for (int i = 0; i < normals.rows(); ++i) {
            double norm = normals.row(i).norm();
            if (norm > 1e-8) {
                normals.row(i) /= norm;
            } else {
                normals.row(i) = Eigen::RowVector3d(0.0, 1.0, 0.0);
            }
        }

        return normals;
    }

    static LayeredMesh mean_mesh(const volumetric_base::Gender& gender = Gender::MALE) {
        pmp::SurfaceMesh skeleton, muscle, skin;

        pmp::read(skeleton, volanim::filenames::mean_layer(Layer::SKEL, gender));
        pmp::read(muscle, volanim::filenames::mean_layer(Layer::MUSC, gender));
        pmp::read(skin, volanim::filenames::mean_layer(Layer::SKIN, gender));

        return { skeleton, muscle, skin };
    };


    static Prism getPrism(int i);

    //Indexing methods and tet order
    static auto Index(pmp::Vertex v, volumetric_base::Layer type) -> long
    {
        return v.idx() * 3 + static_cast<std::underlying_type_t<Layer>>(type);
    }

    static LayerVertex get_vertex(const int vertex_index) {
        return {pmp::Vertex(vertex_index/3), getLayer(vertex_index % 3)};
    }
    [[nodiscard]]
    auto Index(const Quad& quad) const -> long
    {
        const auto& [edge, type] = quad;
        return 3 * static_cast<long>(n_vertices()) + 2 * static_cast<long>(edge.idx()) + static_cast<std::underlying_type_t<Layer>>(type);
    }

    void write(const std::string& path) const
    {
        auto [s1,s2,s3] = toSurfaceMeshes();
        pmp::write(s1, path+"_skeleton_.obj");
        pmp::write(s2, path+"_muscle_.obj");
        pmp::write(s3, path+"_skin_.obj");
    }

    static LayeredMesh read(const std::string& path) {
        pmp::SurfaceMesh skel,musc,skin;
        pmp::read(skel, path+"_skeleton_.obj");
        pmp::read(musc, path+"_muscle_.obj");
        pmp::read(skin, path+"_skin_.obj");
        return {skel,musc, skin};
    }

    static LayeredMesh read(const std::filesystem::path& skel_fn, const std::filesystem::path& musc_fn, const std::filesystem::path& skin_fn)
    {
        pmp::SurfaceMesh skel,musc,skin;
        pmp::read(skel, skel_fn);
        pmp::read(musc, musc_fn);
        pmp::read(skin, skin_fn);
        return {skel,musc, skin};
    }

    LayeredMesh operator+(const LayeredMesh & other) const
    {
        LayeredMesh mesh = other;
        for (const LayerVertex& lv : this->getLayerVertices())
        {
            mesh.position(lv) += this->position(lv);
        }
        return mesh;
    }

    LayeredMesh operator-(const LayeredMesh & other) const
    {
        LayeredMesh mesh = *this;
        for (const LayerVertex& lv : this->getLayerVertices())
        {
            mesh.position(lv) -= other.position(lv);
        }
        return mesh;
    }

    static LayeredMesh template_mesh(const volumetric_base::Gender gender = Gender::MALE)
    {
        pmp::SurfaceMesh skeleton, muscle, skin;

        pmp::read(skeleton, volanim::filenames::template_layer(Layer::SKEL, gender));
        pmp::read(muscle, volanim::filenames::template_layer(Layer::MUSC, gender));
        pmp::read(skin, volanim::filenames::template_layer(Layer::SKIN, gender));

        return { skeleton, muscle, skin };
    }

    [[nodiscard]]
    static auto innerIndex(const Quad& quad) -> long
    {
        const auto& [edge, type] = quad;
        return 2 * static_cast<long>(edge.idx()) + static_cast<std::underlying_type_t<Layer>>(type);
    }

    [[nodiscard]]
    static auto innerIndex(const Prism& cell) -> long
    {
        const auto& [face, type] = cell;
        return 2 * static_cast<long>(face.idx()) + static_cast<std::underlying_type_t<Layer>>(type);
    }

    [[nodiscard]]
    auto Index(const Prism& cell) const -> long
    {
        const auto& [face, type] = cell;
        return 3 * static_cast<long>(n_vertices()) + 2 * static_cast<long>(n_edges()) + 2 * static_cast<long>(face.idx()) + static_cast<std::underlying_type_t<Layer>>(type);
    }

    [[nodiscard]] bool is_cleaned() const {
        for (pmp::Vertex v : vertices()) {
            if (is_boundary(v)) {
                return true;
            }
        }
        return false;
    }


    static auto Index(const std::tuple<pmp::Vertex, Layer>& v) -> long
    {
        const auto& [vertex, type] = v;
        return Index(vertex, type);
    }

    static std::tuple<int, int, int, int> getTetrahedraOrder(int i1, int i2, int i3, int i4,
                                                             Eigen::Matrix<double, -1, 3>& positions);

    // Vertex-related functions
    [[nodiscard]]
    int n_layer_vertices() const;

    [[nodiscard]]
    int n_virtual_vertices() const;

    [[nodiscard]]
    std::vector<LayerVertex> getLayerVertices() const;

    [[nodiscard]]
    std::tuple<std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>>
    getLayerVertices(Quad quad) const;

    [[nodiscard]]
    std::tuple<std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>,
        std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>, std::tuple<pmp::Vertex, Layer>>
    getLayerVertices(Prism cell) const;

    // Position-related functions
    [[nodiscard]]
    Eigen::Vector3d& position(pmp::Vertex v, Layer type) const;

    [[nodiscard]]
    Eigen::Vector3d& position(LayerVertex) const;

    [[nodiscard]]
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    getPositions(Quad quad) const;

    [[nodiscard]]
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    getPositions(Prism cell) const;

    [[nodiscard]]
    Eigen::Matrix<double,3,6> getPositionsAsMatrix(const Prism& cell) const;

    [[nodiscard]]
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
    getGroundPositions(pmp::Face f, Layer type) const;

    [[nodiscard]]
    std::tuple<int, int, int, int>
    getIndices(const Quad& quad) const;

    [[nodiscard]]
    std::tuple<int, int, int, int, int, int>
    getIndices(const Prism& cell) const;
    [[nodiscard]] std::vector<long> getIndicesV(const Prism& cell) const;

    [[nodiscard]]
    int n_prisms() const;

    [[nodiscard]]
    std::vector<std::tuple<int, int, int, int>>
    getTetrahedraIndices(const Prism& cell) const;

    [[nodiscard]]
    std::vector<Quad>
    getQuadsV(const Prism& cell) const;

    [[nodiscard]]
    std::tuple<Quad, Quad, Quad>
    getQuads(const Prism& cell) const;

    [[nodiscard]]
    std::tuple<pmp::Edge, pmp::Edge, pmp::Edge>
    getEdges(pmp::Face f) const;

    [[nodiscard]]
    std::tuple<pmp::Vertex, pmp::Vertex, pmp::Vertex>
    getVertices(pmp::Face f) const;

    [[nodiscard]]
    std::vector<Prism>
    getPrisms() const;


    [[nodiscard]]
    Eigen::Matrix<double, -1, 3>
    toVertexMatrix() const;

    void toVertexMatrix(Eigen::MatrixXd& positions) const;
    void set_mesh_from_matrix(const Eigen::MatrixXd& positions) const;
    auto apply_constrained_update(const Eigen::VectorXd& positions, const std::vector<int> &mapping) const -> void;

    [[nodiscard]]
    Eigen::VectorXd toVertexVector() const;

    void set_mesh_from_vector(Eigen::VectorXd& positions) const;


    // Conversion functions
    [[nodiscard]]
    std::tuple<SurfaceMesh, SurfaceMesh, SurfaceMesh>
    toSurfaceMeshes() const;

    [[nodiscard]]
    auto to_mesh(Layer layer) const -> SurfaceMesh;

    // faster copy operation for new positions
    auto set_positions_from(const LayeredMesh& layered_mesh) const -> void;
    auto set_positions_from(const SurfaceMesh& skeleton, const SurfaceMesh& muscle, const SurfaceMesh& skin) const -> void;
    static auto fast_vertex_matrix(const pmp::SurfaceMesh& skel, const pmp::SurfaceMesh& musc, const pmp::SurfaceMesh& skin, const std::string& prop_name = pmp_character::properties::vertexPosition) -> Eigen::MatrixXd {
        assert(skel.n_vertices() == musc.n_vertices());
        assert(skel.n_vertices() == skin.n_vertices());

        const auto& skel_prop = skel.get_vertex_property<pmp::Point>(prop_name);
        const auto& musc_prop = musc.get_vertex_property<pmp::Point>(prop_name);
        const auto& skin_prop = skin.get_vertex_property<pmp::Point>(prop_name);

        Eigen::Matrix<double, -1, 3> positions(3 * skel.n_vertices(), 3);
        for (const auto v: skel.vertices()) {
            positions.row(Index(v, Layer::SKEL)) = Eigen::Vector3f(skel_prop[v]).cast<double>();
            positions.row(Index(v, Layer::MUSC)) = Eigen::Vector3f(musc_prop[v]).cast<double>();
            positions.row(Index(v, Layer::SKIN)) = Eigen::Vector3f(skin_prop[v]).cast<double>();
        }
        return positions;
    }
private:
    std::vector<std::tuple<int, int, int, int> > tets;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif // LAYEREDMESH_H

// =====================================================================================================================
