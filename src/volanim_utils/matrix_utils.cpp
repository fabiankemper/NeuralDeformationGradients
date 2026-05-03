// =====================================================================================================================

#include "matrix_utils.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------


auto get_translation_part(const pmp::mat4& matrix) -> pmp::Point
{
    return pmp::Point(matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

// ---------------------------------------------------------------------------------------------------------------------

auto set_translation_part(pmp::mat4& matrix, const pmp::Point& translation) -> void
{
    matrix(0, 3) = translation[0];
    matrix(1, 3) = translation[1];
    matrix(2, 3) = translation[2];
}

// ---------------------------------------------------------------------------------------------------------------------

auto mesh_pos_to_matrix(const pmp::SurfaceMesh& mesh) -> Eigen::Matrix3Xf
{
    Eigen::Matrix3Xf result(3, mesh.n_vertices());
    for (auto vertex : mesh.vertices()) {
        const auto& pos = mesh.position(vertex);
        result.col(vertex.idx()) = Eigen::Vector3f(pos[0], pos[1], pos[2]);
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto mesh_pos_from_matrix(pmp::SurfaceMesh& mesh, const Eigen::Matrix3Xf& matrix) -> void
{
    assert(mesh.n_vertices() == matrix.cols());

    for (Eigen::Index idx = 0; idx < static_cast<Eigen::Index>(mesh.n_vertices()); ++idx) {
        mesh.position(pmp::Vertex(idx)) = matrix.col(idx);
    }
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================
