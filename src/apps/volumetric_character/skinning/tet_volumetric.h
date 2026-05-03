// =====================================================================================================================
// Created by fabian on 9/19/24.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef TET_VOLUMETRIC_SKINNING_H
#define TET_VOLUMETRIC_SKINNING_H

// =====================================================================================================================

#include "lbs.h"
#include "volanim_utils/defs.h"

#include "volumetric_skinning/base/skinning_base.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

class TetVolumetricSkinning : public LinearBlendSkinning {
public:
    TetVolumetricSkinning();
    ~TetVolumetricSkinning() override;

    // updated pose
    auto updated_pose(volumetric_base::VolumetricState* state) -> void override;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //TET_VOLUMETRIC_SKINNING_H

// =====================================================================================================================
