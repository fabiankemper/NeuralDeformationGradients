// =====================================================================================================================

#pragma once
#ifndef PMPCHARACTER_CONSTANTS_H
#define PMPCHARACTER_CONSTANTS_H

// =====================================================================================================================

#include <filesystem>
#include <string>

#include "volumetric_base/state/filenames.h"
#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::config)

// =====================================================================================================================

// samples dir
#define SAMPLES_SKELS "skel"
#define SAMPLES_MUSCS "musc"
#define SAMPLES_SKINS "skin"

#define SAMPLES_FILE_EXT ".obj"

// configuration parameters, read at program start
extern const std::filesystem::path current_work_dir;

// character node
extern const std::string layer_skel_wrap_name;
extern const std::string layer_skin_name;


// =====================================================================================================================

NAMESPACE_END(volanim::config)

// =====================================================================================================================

#endif //PMPCHARACTER_CONSTANTS_H

// =====================================================================================================================
