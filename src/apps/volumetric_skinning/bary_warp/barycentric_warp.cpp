// =====================================================================================================================
// Created by fabian on 5/16/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "barycentric_warp.h"
#include "bary_warp.h"
#include "fbxsdk/core/fbxsystemunit.h"

// =====================================================================================================================

NAMESPACE_BEGIN(bary_warp)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

BarycentricWarp::BarycentricWarp() = default;

// ---------------------------------------------------------------------------------------------------------------------

BarycentricWarp::~BarycentricWarp()
{
    delete _barycentric_warper;
}

// ---------------------------------------------------------------------------------------------------------------------

auto BarycentricWarp::initialize(volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("BarycentricWarp::initialize(): nullptr");
    }
    _template_mesh = volumetric_base::LayeredMesh(state->get_highres_wrap(volumetric_base::Layer::SKEL), state->get_highres_wrap(volumetric_base::Layer::MUSC), state->get_highres_wrap(volumetric_base::Layer::SKIN));

    const auto& hires_skel_mesh = state->get_highres_mesh(volumetric_base::Layer::SKEL);
    const auto& hires_musc_mesh = state->get_highres_mesh(volumetric_base::Layer::MUSC);

    delete _barycentric_warper;
    _barycentric_warper = new BarycentricWarper(_template_mesh, hires_skel_mesh, hires_musc_mesh, state->gender);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(bary_warp)

// =====================================================================================================================

