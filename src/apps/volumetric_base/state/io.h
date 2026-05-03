// =====================================================================================================================
//
// Created by Fabian Kemper on 2025-04-10.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_STATE_IO_H
#define VOLUMETRIC_STATE_IO_H

// =====================================================================================================================

#include <filesystem>

#include "volanim_utils/defs.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(io)

// =====================================================================================================================

using Path = std::filesystem::path;

// =====================================================================================================================

// read high-res meshes from fbx file
auto state_read_highres_mesh(VolumetricState* state, Layer layer, const Path& filename, const std::string& mesh_name) -> void;

// read high-res template wraps (for high-res meshes)
auto state_read_highres_wrap(VolumetricState& state, Layer layer, const Path& path) -> void;

// set skinning parameters from npz
auto read_skinning_parameters_from_npz(VolumetricState* state, const Path& filename) -> void;

// store skinning parameters as npz
auto write_skinning_parameters_as_npz(VolumetricState* state, const Path& filename) -> void;

// read skeleton rig from file
auto read_rig_from_file(VolumetricState* state, const Path& filename) -> void;

// write current rig to file
auto write_rig_to_file(VolumetricState* state, const Path& filename) -> void;

// =====================================================================================================================

NAMESPACE_END(io)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_STATE_IO_H

// =====================================================================================================================
