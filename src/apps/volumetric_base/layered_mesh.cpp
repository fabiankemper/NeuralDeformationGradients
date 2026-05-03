// =====================================================================================================================

#include "layered_mesh.h"

#include <pmp/stop_watch.h>
#include <pmp_character/utility/VertexDuplicationUtilities.h>

#include "diffgeo/TriTriIntersect.h"
#include "diffgeo/prism_diffgeo.h"
#include "utility/constants.h"
#include "utility/matrix_util.hpp"
#include "volanim_utils/io/io_selection.h"
// =====================================================================================================================


NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

#define MEASURE_TIME false

// =====================================================================================================================

using namespace pmp;
using namespace Eigen;
using namespace volumetric_base;
// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

bool anyPointsAreTheSame(const Vector3d& p1, const Vector3d& p2, const Vector3d& p3,
                         const Vector3d& p4, const Vector3d& p5, const Vector3d& p6) {
    Vector3d points[6] = {p1, p2, p3, p4, p5, p6};

    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = i + 1; j < 6; ++j) {
            if ((points[i] - points[j]).norm() < 0.000001) {
                return true;
            }
        }
    }

    return false;
}


// ---------------------------------------------------------------------------------------------------------------------

double signedAreaOfTriangle(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3) {
    Vector3d side1 = v2 - v1;
    Vector3d side2 = v3 - v1;
    Vector3d crossProduct = side1.cross(side2);
    double area = 0.5 * crossProduct.norm();
    double sign = (crossProduct.z() >= 0) ? 1.0 : -1.0;
    return sign * area;
}


// ---------------------------------------------------------------------------------------------------------------------

LayeredMesh::LayeredMesh(const SurfaceMesh& skeleton, const SurfaceMesh& muscle,
                         const SurfaceMesh& skin) : pmp::SurfaceMesh(skeleton) {
    initialize(skeleton, muscle, skin);
}

// ---------------------------------------------------------------------------------------------------------------------

void LayeredMesh::initialize(const SurfaceMesh& skeleton, const SurfaceMesh& muscle, const SurfaceMesh& skin) {
    VertexProperty<Vector3d> skeletonPosition = this->add_vertex_property<Vector3d>(VP_SKELETON_POSITION, Eigen::Vector3d::Zero());
    VertexProperty<Vector3d> musclePosition = this->add_vertex_property<Vector3d>(VP_MUSCLE_POSITION,Eigen::Vector3d::Zero());
    VertexProperty<Vector3d> skinPosition = this->add_vertex_property<Vector3d>(VP_SKIN_POSITION, Eigen::Vector3d::Zero());

    for (Vertex v: vertices()) {
        skeletonPosition[v] = skeleton.position(v);
        musclePosition[v] = muscle.position(v);
        skinPosition[v] = skin.position(v);
    }
}

//----------------------------------------------------------------------------------------------------------------------

Prism LayeredMesh::getPrism(int i) {
    return {pmp::Face(i/2), static_cast<Layer>(i % 2)};
}



// ---------------------------------------------------------------------------------------------------------------------

        Eigen::Matrix<double, -1, 3> LayeredMesh::toVertexMatrix() const {
            Eigen::Matrix<double, -1, 3> positions(3 * n_vertices(), 3);
            for (Vertex v: vertices()) {
                positions.row(Index(v, Layer::SKEL)) = position(v, Layer::SKEL);
                positions.row(Index(v, Layer::MUSC)) = position(v, Layer::MUSC);
                positions.row(Index(v, Layer::SKIN)) = position(v, Layer::SKIN);
            }

            return positions;
        }

// ---------------------------------------------------------------------------------------------------------------------

void LayeredMesh::toVertexMatrix(MatrixXd& positions) const {
    positions = MatrixXd(3 * n_vertices(), 3);
    for (const Vertex v: vertices()) {
        if (static_cast<unsigned int>(Index(v, Layer::SKEL) < 0 || Index(v, Layer::SKEL)) > 3 * n_vertices()) {
            std::cout << "MAJOR ERROR " << std::endl;
        }
        positions.row(Index(v, Layer::SKEL)) = position(v, Layer::SKEL);
        positions.row(Index(v, Layer::MUSC)) = position(v, Layer::MUSC);
        positions.row(Index(v, Layer::SKIN)) = position(v, Layer::SKIN);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

Layer getNextLayer(Layer inner) {
    if (inner == Layer::SKEL)
        return Layer::MUSC;
    if (inner == Layer::MUSC)
        return Layer::SKIN;

    std::cout << "ERROR IN GETTING NEXT MESH TYPE (INPUT IS " << static_cast<std::underlying_type_t<Layer>>(inner) << std::endl;
    exit(0);
}

// ---------------------------------------------------------------------------------------------------------------------

Vector3d& LayeredMesh::position(Vertex v, Layer type) const {
    VertexProperty<Vector3d> posis;
    if (type == Layer::SKEL) {
        posis = this->get_vertex_property<Vector3d>(VP_SKELETON_POSITION);
        return posis[v];
    }
    if (type == Layer::MUSC) {
        posis = this->get_vertex_property<Vector3d>(VP_MUSCLE_POSITION);
        return posis[v];
    }
    posis = this->get_vertex_property<Vector3d>(VP_SKIN_POSITION);
    return posis[v];
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Vector3d& LayeredMesh::position(LayerVertex v) const {
    auto [vertex, type] = v;
    return position(vertex, type);
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Vertex, Vertex, Vertex> LayeredMesh::getVertices(Face f) const {
    auto iterator = vertices(f);
    Vertex v1 = *iterator++;
    Vertex v2 = *iterator++;
    Vertex v3 = *iterator;

    return {v1, v2, v3};
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Vector3d, Vector3d, Vector3d> LayeredMesh::getGroundPositions(Face f, Layer type) const {
    auto [v1,v2,v3] = getVertices(f);
    Vector3d p1 = position(v1, type);
    Vector3d p2 = position(v2, type);
    Vector3d p3 = position(v3, type);
    return {p1, p2, p3};
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<Prism> LayeredMesh::getPrisms() const {
    std::vector<Prism> result;
    for (Face f: faces()) {
        result.emplace_back(f, Layer::SKEL);
        result.emplace_back(f, Layer::MUSC);
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<int, int, int, int> LayeredMesh::getIndices(const Quad& quad) const {
    auto [edge, inner] = quad;
    Layer outer = getNextLayer(inner);
    Vertex v1 = vertex(edge, 0);
    Vertex v2 = vertex(edge, 1);
    return {Index(v1, inner), Index(v2, inner), Index(v1, outer), Index(v2, outer)};
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<int, int, int, int, int, int> LayeredMesh::getIndices(const Prism& cell) const {
    auto [face, inner] = cell;
    Layer outer = getNextLayer(inner);
    auto [v1,v2,v3] = getVertices(face);
    return {
        Index(v1, inner), Index(v2, inner), Index(v3, inner), Index(v1, outer), Index(v2, outer), Index(v3, outer)
    };
}


//---------------------------------------------------------------------------------------------------------------------

std::vector<long> LayeredMesh::getIndicesV(const Prism& cell) const {
    auto [face, inner] = cell;
    Layer outer = getNextLayer(inner);
    auto [v1,v2,v3] = getVertices(face);
    return {Index(v1, inner), Index(v2, inner), Index(v3, inner), Index(v1, outer), Index(v2, outer), Index(v3, outer)
    };
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<LayerVertex> LayeredMesh::getLayerVertices() const {
    std::vector<LayerVertex> verts;
    for (Vertex v: vertices()) {
        verts.emplace_back(v, Layer::SKEL);
        verts.emplace_back(v, Layer::MUSC);
        verts.emplace_back(v, Layer::SKIN);
    }

    return verts;
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Edge, Edge, Edge> LayeredMesh::getEdges(Face f) const {
    auto iterator = halfedges(f);
    Halfedge h1 = *iterator++;
    Halfedge h2 = *iterator++;
    Halfedge h3 = *iterator;
    return {edge(h1), edge(h2), edge(h3)};
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Quad, Quad, Quad> LayeredMesh::getQuads(const Prism& cell) const {
    auto [face, type] = cell;
    auto [e1,e2,e3] = getEdges(face);
    return {Quad(e1, type), Quad(e2, type), Quad(e3, type)};
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<Quad> LayeredMesh::getQuadsV(const Prism& cell) const {
    auto [face, type] = cell;
    auto [e1,e2,e3] = getEdges(face);
    return {Quad(e1, type), Quad(e2, type), Quad(e3, type)};
}

// ---------------------------------------------------------------------------------------------------------------------

VectorXd LayeredMesh::toVertexVector() const {
    VectorXd positions(n_vertices() * 3 * 3);
    for (LayerVertex v: getLayerVertices()) {
        int idx = static_cast<int>(Index(v));
        positions.block<3, 1>(3 * idx, 0) = position(v);
    }
    return positions;
}



// ---------------------------------------------------------------------------------------------------------------------

int LayeredMesh::n_prisms() const {
    return 2 * static_cast<int>(n_faces());
}
// ---------------------------------------------------------------------------------------------------------------------

int LayeredMesh::n_virtual_vertices() const {
    return 3 * static_cast<int>(n_vertices()) + 2 * static_cast<int>(n_faces()) + 2 * static_cast<int>(n_edges());
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Vector3d, Vector3d, Vector3d, Vector3d> LayeredMesh::getPositions(Quad quad) const {
    auto [e, innertype] = std::move(quad);
    Layer outertype = getNextLayer(innertype);
    Vertex v1 = vertex(e, 0);
    Vertex v2 = vertex(e, 1);

    return {position(v1, innertype), position(v2, innertype), position(v1, outertype), position(v2, outertype)};
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<LayerVertex, LayerVertex, LayerVertex, LayerVertex> LayeredMesh::getLayerVertices(Quad quad) const {
    auto [e, innertype] = std::move(quad);
    Layer outertype = getNextLayer(innertype);
    Vertex v1 = vertex(e, 0);
    Vertex v2 = vertex(e, 1);

    return {{v1, innertype}, {v2, innertype}, {v1, outertype}, {v2, outertype}};
}

// ---------------------------------------------------------------------------------------------------------------------

int LayeredMesh::n_layer_vertices() const {
    return 3 * static_cast<int>(n_vertices());
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<int, int, int, int> LayeredMesh::getTetrahedraOrder(int i1, int i2, int i3, int i4,
                                                               Eigen::Matrix<double, -1, 3>& positions) {
    // Get the positions of the vertices
    auto position = [&](int i) -> Vector3d { return positions.row(i); };
    Eigen::Vector3d p1 = position(i1);
    Eigen::Vector3d p2 = position(i2);
    Eigen::Vector3d p3 = position(i3);
    Eigen::Vector3d p4 = position(i4);

    Eigen::Matrix4d matrix;
    matrix << p1.x(), p1.y(), p1.z(), 1,
            p2.x(), p2.y(), p2.z(), 1,
            p3.x(), p3.y(), p3.z(), 1,
            p4.x(), p4.y(), p4.z(), 1;

    double determinant = matrix.determinant();

    if (determinant < 0) {
        return std::make_tuple(i1, i2, i3, i4);
    } else {
        return std::make_tuple(i1, i2, i4, i3);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<int, int, int, int> getVertexIndices(const LayeredMesh& mesh, const Quad& quad)
{
    auto [v1,v2,v3,v4] = mesh.getLayerVertices(quad);
    return {LayeredMesh::Index(v1), LayeredMesh::Index(v2), LayeredMesh::Index(v3), LayeredMesh::Index(v4)};
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<std::tuple<int, int, int, int> > LayeredMesh::getTetrahedraIndices(const Prism& cell) const {
    auto [ground_face, inner] = cell;
    Layer outer = getNextLayer(inner);
    std::vector<std::tuple<int, int, int, int> > tets_;
    unsigned int cellPoint = Index(cell);
    for (Halfedge hedge: halfedges(ground_face)) {
        Edge e = edge(hedge);
        Quad quad(e, inner);
        auto [p1,p2,p3,p4] = getIndices(quad);
        unsigned int quadPoint = Index(quad);

        tets_.emplace_back(p1, p2, quadPoint, cellPoint);
        tets_.emplace_back(p1, p3, quadPoint, cellPoint);
        tets_.emplace_back(p2, p3, quadPoint, cellPoint);
        tets_.emplace_back(p3, p4, quadPoint, cellPoint);
    }
    auto [v1, v2, v3] = getVertices(ground_face);


    tets_.emplace_back(Index(v1, inner), Index(v2, inner), Index(v3, inner), cellPoint);
    tets_.emplace_back(Index(v1, outer), Index(v2, outer), Index(v3, outer), cellPoint);
    return tets_;
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<Vector3d, Vector3d, Vector3d, Vector3d, Vector3d, Vector3d> LayeredMesh::getPositions(Prism cell) const {
    auto [face, type] = cell;

    auto [p1,p2,p3] = getGroundPositions(face, type);
    auto [p1_,p2_,p3_] = getGroundPositions(face, getNextLayer(type));

    return {p1, p2, p3, p1_, p2_, p3_};
}

Eigen::Matrix<double, 3, 6> LayeredMesh::getPositionsAsMatrix(const Prism &cell) const {
    auto [face, type] = cell;

    auto [p1,p2,p3] = getGroundPositions(face, type);
    auto [p1_,p2_,p3_] = getGroundPositions(face, getNextLayer(type));
    Eigen::Matrix<double,3,6> posis;
    posis.col(0) = p1;
    posis.col(1) = p2;
    posis.col(2) = p3;
    posis.col(3) = p1_;
    posis.col(4) = p2_;
    posis.col(5) = p3_;
    return posis;
}

// ---------------------------------------------------------------------------------------------------------------------

std::tuple<LayerVertex, LayerVertex, LayerVertex, LayerVertex, LayerVertex, LayerVertex> LayeredMesh::getLayerVertices(
    Prism cell) const {
    auto [face, inner] = std::move(cell);
    Layer outer = getNextLayer(inner);
    auto [v1,v2,v3] = getVertices(face);

    return {
        LayerVertex(v1, inner), LayerVertex(v2, inner), LayerVertex(v3, inner), LayerVertex(v1, outer),
        LayerVertex(v2, outer), LayerVertex(v3, outer)
    };
}

// ---------------------------------------------------------------------------------------------------------------------

auto LayeredMesh::set_mesh_from_vector(Eigen::VectorXd& positions) const -> void {
    for (LayerVertex v: getLayerVertices()) {
        position(v) = positions.block<3, 1>(Index(v) * 3, 0);
    }
}

auto LayeredMesh::apply_constrained_update(const Eigen::VectorXd& positions, const std::vector<int> &mapping) const -> void {
    for (LayerVertex v: getLayerVertices()) {
        if (mapping[Index(v)] !=-1)
            position(v) = position(v)+positions.block<3, 1>(mapping[Index(v)] * 3, 0);
    }
}


void LayeredMesh::set_mesh_from_matrix(const Eigen::MatrixXd& positions) const {
    for (Vertex v: vertices()) {
         position(v, Layer::SKEL) = positions.row(Index(v, Layer::SKEL));
         position(v, Layer::MUSC) = positions.row(Index(v, Layer::MUSC));
         position(v, Layer::SKIN) = positions.row(Index(v, Layer::SKIN));
    }
}


// ---------------------------------------------------------------------------------------------------------------------

std::tuple<SurfaceMesh, SurfaceMesh, SurfaceMesh> LayeredMesh::toSurfaceMeshes() const {
    SurfaceMesh skeleton, muscle, skin;
    skeleton.assign(*this);
    muscle.assign(*this);
    skin.assign(*this);

    for (Vertex v: vertices()) {
        skeleton.position(v) = position(v, Layer::SKEL);
        muscle.position(v) = position(v, Layer::MUSC);
        skin.position(v) = position(v, Layer::SKIN);
    }

    if (this->has_vertex_property(VP_OLD_INDICES)) {
        auto my_indices =this->get_vertex_property<pmp::IndexType>(VP_OLD_INDICES);
        auto skel_indices = skeleton.add_vertex_property<pmp::IndexType>(VP_OLD_INDICES);
        for (pmp::Vertex v: vertices()) {
            skel_indices[v] =my_indices[v];
        }
    }
    return {skeleton, muscle, skin};
}

// ---------------------------------------------------------------------------------------------------------------------

auto LayeredMesh::to_mesh(const Layer layer) const -> SurfaceMesh
{
    SurfaceMesh result;
    result.assign(*this); // connectivity

    if (!has_vertex_property(VP_SKELETON_POSITION) || !has_vertex_property(VP_MUSCLE_POSITION) || !has_vertex_property(VP_SKIN_POSITION)) {
        std::cerr << "[ERROR] Illegal state when converting layered mesh to surface mesh.\n";
        return result;
    }

    VertexProperty<Vector3d> vp_src;

    switch (layer) {
        case Layer::SKEL:
            vp_src = get_vertex_property<Eigen::Vector3d>(VP_SKELETON_POSITION);
            break;
        case Layer::MUSC:
            vp_src = get_vertex_property<Eigen::Vector3d>(VP_MUSCLE_POSITION);
            break;
        case Layer::SKIN:
            vp_src = get_vertex_property<Eigen::Vector3d>(VP_SKIN_POSITION);
            break;
    }

    for (const Vertex v: vertices()) {
        result.position(v) = vp_src[v].cast<float>();
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto LayeredMesh::set_positions_from(const LayeredMesh& layered_mesh) const -> void
{
    if (this->n_vertices() != layered_mesh.n_vertices()) {
        throw std::runtime_error("LayeredMesh::set_positions_from: wrong number of vertices");
    }

    auto vp_skel_src = layered_mesh.get_vertex_property<pmp::Point>(VP_SKELETON_POSITION);
    auto vp_musc_src = layered_mesh.get_vertex_property<pmp::Point>(VP_MUSCLE_POSITION);
    auto vp_skin_src =  layered_mesh.get_vertex_property<pmp::Point>(VP_SKIN_POSITION);

    auto vp_skel_dest = this->get_vertex_property<pmp::Point>(VP_SKELETON_POSITION);
    auto vp_musc_dest = this->get_vertex_property<pmp::Point>(VP_MUSCLE_POSITION);
    auto vp_skin_dest =  this->get_vertex_property<pmp::Point>(VP_SKIN_POSITION);

    size_t n_vertices = layered_mesh.n_vertices();

    std::memmove(vp_skel_dest.vector().data(), vp_skel_src.data(), sizeof(pmp::Point) * n_vertices);
    std::memmove(vp_musc_dest.vector().data(), vp_musc_src.data(), sizeof(pmp::Point) * n_vertices);
    std::memmove(vp_skin_dest.vector().data(), vp_skin_src.data(), sizeof(pmp::Point) * n_vertices);
}

// ---------------------------------------------------------------------------------------------------------------------

auto LayeredMesh::set_positions_from(
    const SurfaceMesh& skeleton,
    const SurfaceMesh& muscle,
    const SurfaceMesh& skin
) const -> void
{
    if (this->n_vertices() != skeleton.n_vertices() || this->n_vertices() != muscle.n_vertices() || this->n_vertices() != skin.n_vertices()) {
        throw std::runtime_error("LayeredMesh::set_positions_from: wrong number of vertices");
    }

    auto vp_skel = this->get_vertex_property<pmp::Point>(VP_SKELETON_POSITION);
    auto vp_musc = this->get_vertex_property<pmp::Point>(VP_MUSCLE_POSITION);
    auto vp_skin =  this->get_vertex_property<pmp::Point>(VP_SKIN_POSITION);

    auto skel_pos = skeleton.get_vertex_property<pmp::Point>(VP_POSITION);
    auto musc_pos = skeleton.get_vertex_property<pmp::Point>(VP_POSITION);
    auto skin_pos = skeleton.get_vertex_property<pmp::Point>(VP_POSITION);

    std::memmove(vp_skel.vector().data(), skel_pos.data(), sizeof(pmp::Point) * skeleton.n_vertices());
    std::memmove(vp_musc.vector().data(), musc_pos.data(), sizeof(pmp::Point) * muscle.n_vertices());
    std::memmove(vp_skin.vector().data(), skin_pos.data(), sizeof(pmp::Point) * skin.n_vertices());
}



// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================
