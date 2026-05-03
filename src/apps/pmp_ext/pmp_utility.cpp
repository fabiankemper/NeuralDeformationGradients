// =====================================================================================================================

#include "pmp_ext/pmp_utility.h"

#include <fstream>

#include <pmp/algorithms/differential_geometry.h>
#include <pmp_character/PropertyNames.h>

#include "pmp_ext/mesh_properties.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext::utils)

// =====================================================================================================================

namespace MeshProperties = pmp_ext::properties;

// =====================================================================================================================

double edgeAreaOld(pmp::SurfaceMesh& mesh, pmp::Edge e)
{
    auto he = mesh.halfedge(e, 0);
    auto vp = mesh.from_vertex(he);
    auto vq = mesh.to_vertex(he);
    auto vr = mesh.to_vertex(mesh.next_halfedge(he));
    auto vs = mesh.to_vertex(mesh.next_halfedge(mesh.opposite_halfedge(he)));

    auto xp = mesh.position(vp);
    auto xq = mesh.position(vq);
    auto xr = mesh.position(vr);
    auto xs = mesh.position(vs);

    double area = (pmp::triangle_area(xp, xq, xr) + pmp::triangle_area(xp, xq, xs)) / 3.0;
    return area;
}

void minimizeVectorCoordinates(pmp::vec3& ref, const pmp::vec3& o)
{
    if (o[0] < ref[0]) ref[0] = o[0];
    if (o[1] < ref[1]) ref[1] = o[1];
    if (o[2] < ref[2]) ref[2] = o[2];
}

void maximizeVectorCoordinates(pmp::vec3& ref, const pmp::vec3& o)
{
    if (o[0] > ref[0]) ref[0] = o[0];
    if (o[1] > ref[1]) ref[1] = o[1];
    if (o[2] > ref[2]) ref[2] = o[2];
}

void meanCenterTemplateToPointcloud(const pmp_character::Character& character, pmp_character::PointSet& point_set)
{
    pmp::BoundingBox box = point_set.bounds();
    character.skeleton_.root_->global_ += translation_matrix(box.center());
}

void getFaceVertices(const pmp::SurfaceMesh& mesh, const pmp::Face face, std::vector<pmp::Vertex>& vertices)
{
    for (pmp::Vertex v: mesh.vertices(face)) {
        vertices.emplace_back(v);
    }
}

void getFaceVertexPositions(const pmp::SurfaceMesh& mesh, pmp::Face face, std::vector<pmp::vec3>& vertices,
                            const std::string& property)
{
    auto vertexProperty = mesh.get_vertex_property<pmp::vec3>(property);
    for (pmp::Vertex v: mesh.vertices(face)) {
        vertices.emplace_back(vertexProperty[v]);
    }
}

void flipUVCoordinates(const pmp::SurfaceMesh& mesh)
{
    // Flip UVs for Metashape
    auto uvs = mesh.get_halfedge_property<pmp::vec2>(pmp_character::properties::halfedgeTextureCoordinates);
    for (auto he: mesh.halfedges()) {
        uvs[he][1] = 1 - uvs[he][1];
    }
}

void transformVec3Property(pmp::SurfaceMesh& mesh, const pmp::mat4& transform, const std::string& propertyName)
{
    auto property = mesh.get_vertex_property<pmp::vec3>(propertyName);
    for (auto v: mesh.vertices()) {
        mesh.position(v) = pmp::affine_transform(transform, property[v]);
    }
}

std::vector<double> loadVertexProperty(const std::filesystem::path& propertyPath, pmp::SurfaceMesh& mesh,
                                       std::string propertyName_)
{
    std::vector<double> property;
    std::ifstream file(propertyPath);
    std::string line;
    pmp::VertexProperty<double> vertex_weighting;
    if (not mesh.has_vertex_property(propertyName_)) {
        vertex_weighting = mesh.add_vertex_property<double>(propertyName_, 1);
    } else {
        vertex_weighting = mesh.get_vertex_property<double>(propertyName_);
    }
    while ((std::getline(file, line))) {
        float w = std::stof(line);
        property.emplace_back(w);
    }
    for (size_t i = 0; i < mesh.n_vertices(); i++) {
        vertex_weighting[pmp::Vertex(i)] *= property[i];
    }

    return property;
}

std::vector<double> loadVertexWeights(const std::filesystem::path& weightsPath, pmp::SurfaceMesh& mesh)
{
    return loadVertexProperty(weightsPath, mesh, MeshProperties::vertex_weighting);
}

std::vector<double> loadLaplaceWeights(const std::filesystem::path& weightsPath, pmp::SurfaceMesh& mesh)
{
    return loadVertexProperty(weightsPath, mesh, MeshProperties::laplace_weighting);
}

void addHalfEdgeCotanAttribute(pmp::SurfaceMesh& mesh)
{
    auto cotan = mesh.halfedge_property<double>(MeshProperties::halfedge_cotan);
    for (auto hit: mesh.halfedges()) {
        if (mesh.is_boundary(hit)) {
            cotan[hit] = 0.0;
        } else {
            auto vp = mesh.from_vertex(hit);
            auto vq = mesh.to_vertex(hit);
            auto vr = mesh.to_vertex(mesh.next_halfedge(hit));

            const pmp::Point& xp = mesh.position(vp);
            const pmp::Point& xq = mesh.position(vq);
            const pmp::Point& xr = mesh.position(vr);

            // const double cot = graphene::surface_mesh::cotan(xp-xq, xr-xq);
            // cotan[hit] = std::max( 0.0, clamp_cot(cot) );
            cotan[hit] = pmp::cotan(xp - xq, xr - xq);
        }
    }
}

void addEdgeLaplacianAttribute(pmp::SurfaceMesh& mesh)
{
    auto edgeLaplace = mesh.edge_property<pmp::vec3>(MeshProperties::edge_laplace);
    auto cotan = mesh.halfedge_property<double>(MeshProperties::halfedge_cotan);
    // Recalculate cotans
    addHalfEdgeCotanAttribute(mesh);
    for (auto e: mesh.edges()) {
        auto he = mesh.halfedge(e, 0);
        auto vp = mesh.from_vertex(he);
        auto vq = mesh.to_vertex(he);
        auto vr = mesh.to_vertex(mesh.next_halfedge(he));
        auto vs = mesh.to_vertex(mesh.next_halfedge(mesh.opposite_halfedge(he)));

        auto xp = mesh.position(vp);
        auto xq = mesh.position(vq);
        auto xr = mesh.position(vr);
        auto xs = mesh.position(vs);

        const double area = (pmp::triangle_area(xp, xq, xr) + pmp::triangle_area(xp, xq, xs)) / 3.0;

        if (not mesh.has_vertex_property(MeshProperties::laplace_weighting)) {
            std::cerr << "No laplace weighting found. Fallback to without weighting. " << std::endl;
            return;
        }

        // cotan values for edge-incident angles
        const double wik = cotan[he];
        const double wij = cotan[mesh.prev_halfedge(he)];
        const double wim = cotan[mesh.opposite_halfedge(he)];
        const double wil = cotan[mesh.prev_halfedge(mesh.opposite_halfedge(he))];

        // weights for edge-based mean curvature vector
        double ws = (wil + wim) / area;
        double wp = -(wik + wil) / area;
        double wr = (wik + wij) / area;
        double wq = -(wij + wim) / area;

        // scale to weight smoothing vs. fitting. use sqrt of weight
        // since it will be squared for the energy. energy should be
        // sum_i (area_i * norm(laplace_i)), so weight is sqrt(area_i)
        // wq  *=1.0/area;
        // ws  *=1.0/area;
        // wr  *=1.0/area;
        // wp  *=1.0/area;
        pmp::Point laplace(0, 0, 0);
        laplace += wq * mesh.position(vq);
        laplace += ws * mesh.position(vs);
        laplace += wr * mesh.position(vr);
        laplace += wp * mesh.position(vp);
        edgeLaplace[e] = laplace;
    }
}

// =====================================================================================================================

NAMESPACE_END(pmp_ext::utils)

// =====================================================================================================================
