// =====================================================================================================================
#pragma once
// =====================================================================================================================

#include <filesystem>

#include "../../volumetric_skinning/base/prism_adapter.h"
#include "constants.h"
#include "volanim_utils/io/io_selection.h"
#include "volumetric_base/aliases.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

inline auto add_uncleaned_indices(pmp::SurfaceMesh &mesh) -> void
{
    auto clean_v_ids = mesh.vertex_property<pmp::IndexType>(VP_OLD_INDICES);
    auto clean_f_ids = mesh.face_property<pmp::IndexType>(FP_OLD_INDICES);

    for(const auto v : mesh.vertices()) {
        clean_v_ids[v] = v.idx();
    }

    for (const auto f : mesh.faces()) {
        clean_f_ids[f] = f.idx();
    }
}

inline void clean_mesh(pmp::SurfaceMesh& mesh, const Selection& selection)
{
    for (pmp::Vertex v: mesh.vertices()) {
        if (mesh.is_boundary(v)) {
            std::cerr << "Cleaning a mesh that looks to be already cleaned! Writing mesh into error.off" << std::endl;
            pmp::write(mesh, "error.off");
            exit(3);
        }
    }
    add_uncleaned_indices(mesh);

    for(const auto id : selection) {
        mesh.delete_vertex(pmp::Vertex{id});
    }

    mesh.garbage_collection();
}

inline void cleanMesh(pmp::SurfaceMesh &mesh)
{
    // DEPRECATED, this uses a hard coded directory
    const auto selection_fn = volanim::config::settings.data_dir / "template" / "head_hands_toes.sel";
    const auto vertex_ids = volanim::utils::read_selection(selection_fn);
    clean_mesh(mesh, vertex_ids);
}

inline std::tuple<pmp::Vertex, pmp::Vertex, pmp::Vertex> getFaceVertices(const pmp::SurfaceMesh &mesh, const pmp::Face &f)
{
    auto iterator = mesh.vertices(f);
    pmp::Vertex v1 = *iterator++;
    pmp::Vertex v2 = *iterator++;
    pmp::Vertex v3 = *iterator;
    return {v1,v2,v3};
}

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================
