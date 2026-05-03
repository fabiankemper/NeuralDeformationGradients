// =====================================================================================================================

#ifndef VOLUMETRIC_ANIMATION_ALIASES_H
#define VOLUMETRIC_ANIMATION_ALIASES_H

// =====================================================================================================================

#include <filesystem>
#include <pmp_character/node_types/Character.h>

#include "volanim_utils/defs.h"
#include "volanim_utils/io/io_selection.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

using Path = std::filesystem::path;

using Character = pmp_character::Character;
using Joint = pmp_character::Joint;
using Selection = volanim::utils::Selection;
using Skeleton = pmp_character::Skeleton;

using SurfaceMesh = pmp::SurfaceMesh;

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_ANIMATION_ALIASES_H

// =====================================================================================================================