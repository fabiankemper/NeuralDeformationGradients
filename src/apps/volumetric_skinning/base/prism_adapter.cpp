// =====================================================================================================================

#include "prism_adapter.h"

#include "volumetric_base/utility/utility.h"

// =====================================================================================================================

NAMESPACE_BEGIN(prism_adapter)

// =====================================================================================================================

using namespace pmp;

// =====================================================================================================================

auto transfer_to_surface_mesh(pmp::SurfaceMesh &target_mesh, volumetric_base::LayeredMesh &layeredMesh, volumetric_base::Layer type) -> void
{
    ASSERT(layeredMesh.has_vertex_property(volumetric_base::VP_OLD_INDICES))

    VertexProperty old_indices = layeredMesh.vertex_property<pmp::IndexType>(volumetric_base::VP_OLD_INDICES);

    for(Vertex v: layeredMesh.vertices()) {
        Vertex oldV(old_indices[v]);
         target_mesh.position(oldV) = layeredMesh.position(v, type);
    }
}

// =====================================================================================================================

NAMESPACE_END(prism_adapter)

// =====================================================================================================================
