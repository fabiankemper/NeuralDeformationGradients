#pragma once
#ifndef PRISM_ADAPTER_H
#define PRISM_ADAPTER_H

#include "pmp/surface_mesh.h"
#include "volanim_utils/defs.h"
#include "volumetric_base/layered_mesh.h"


NAMESPACE_BEGIN(prism_adapter)


auto transfer_to_surface_mesh(pmp::SurfaceMesh &target_mesh, volumetric_base::LayeredMesh &layeredMesh, volumetric_base::Layer type) -> void;



NAMESPACE_END(PrismAdapter)


#endif //PRISM_ADAPTER_H
