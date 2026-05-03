// =====================================================================================================================

#pragma once
#ifndef PMP_EXTPMP_IO_H
#define PMP_EXTPMP_IO_H

// =====================================================================================================================

#include "pmp_ext/defs.h"

#include <filesystem>

#include <pmp/io/io_flags.h>
#include <pmp/surface_mesh.h>

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

using Path = std::filesystem::path;
using String = std::string;

using IOFlags = pmp::IOFlags;
using SurfaceMesh = pmp::SurfaceMesh;

// =====================================================================================================================


auto write_mesh_pos_property(const SurfaceMesh& mesh, const Path& filename, const String& property_name) -> void;

auto write_small_obj(const SurfaceMesh& mesh, const std::filesystem::path& filename, IOFlags io_flags = IOFlags{}) -> void;


// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================

#endif //PMP_EXTPMP_IO_H

// =====================================================================================================================
