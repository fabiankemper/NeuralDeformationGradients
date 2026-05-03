// =====================================================================================================================
//
// Created by fabian on 16/04/25.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_BASE_STATE_SKINNING_SKINNING_H
#define VOLUMETRIC_BASE_STATE_SKINNING_SKINNING_H

// =====================================================================================================================

#include <iosfwd>
#include <iosfwd>
#include <vector>
#include <vector>

#include "pmp_ext/defs.h"

#include "volumetric_base/skinnable_mesh.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(skinning)

// =====================================================================================================================

// shortcut, if you don't know which method to call
auto lbs(const VolumetricState* state, SkinnableMesh& skinnable_mesh, Layer fallback_weights_layer = Layer::SKIN) -> void;
auto dqs(const VolumetricState* state, SkinnableMesh& skinnable_mesh, Layer fallback_weights_layer = Layer::SKIN) -> void;

// set skinning weights to layer
auto add_skinning_weights(SkinnableMesh& skinnable_mesh, const SkinningParameters& parameters) -> void;


// perform lbs skinning for this mesh
auto lbs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig) -> void;
auto dqs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig) -> void;

// lbs and dqs when there are no skinning weights in the given layer
auto lbs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig, const SkinningParameters& skinning_parameters) -> void;
auto dqs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig, const SkinningParameters& skinning_parameters) -> void;

// =====================================================================================================================

NAMESPACE_END(skinning)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_BASE_STATE_SKINNING_SKINNING_H

// =====================================================================================================================
