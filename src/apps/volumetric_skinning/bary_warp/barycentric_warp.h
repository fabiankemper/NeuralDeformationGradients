// =====================================================================================================================
// Created by fabian on 5/16/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef BARYCENTRIC_WARP_H
#define BARYCENTRIC_WARP_H

// =====================================================================================================================

#include "bary_warp.h"
#include "volumetric_base/skinnable_mesh.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(bary_warp)


// =====================================================================================================================


class BarycentricWarp {
public:
    BarycentricWarp();
    ~BarycentricWarp();

    // initialize internal matrices
    auto initialize(volumetric_base::VolumetricState* state) -> void;

    volumetric_base::LayeredMesh _template_mesh;
    BarycentricWarper* _barycentric_warper = nullptr;
};



// =====================================================================================================================

NAMESPACE_END(bary_warp)

// =====================================================================================================================

#endif //BARYCENTRIC_WARP_H

// =====================================================================================================================
