#include "skinning_base.h"
NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

SkinningBase::SkinningBase() = default;
SkinningBase::~SkinningBase() =default;

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::updated_mean_layers(
    [[maybe_unused]] volumetric_base::VolumetricState* state
) -> void
{
    std::cerr << "[WARN] SkinningBase::updated_mean_layers not overwritten\n";
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::updated_input_layers(
    [[maybe_unused]] volumetric_base::VolumetricState* state
) -> void
{
    std::cerr << "[WARN] SkinningBase::updated_input_layers not overwritten\n";
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::updated_pose([[maybe_unused]] volumetric_base::VolumetricState* state) -> void
{
    std::cerr << "[WARN] SkinningBase::updated_pose not overwritten\n";
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::perform_skinning(
    [[maybe_unused]] volumetric_base::VolumetricState* state,
    [[maybe_unused]] volumetric_base::SkinnableMesh& skel,
    [[maybe_unused]] volumetric_base::SkinnableMesh& musc,
    [[maybe_unused]] volumetric_base::SkinnableMesh& skin,
    [[maybe_unused]] bool converge
) -> void
{
    throw std::runtime_error("SkinningBase::perform_skinning() must be overwritten.");
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::wake_up() -> void
{
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::imgui_update(
    [[maybe_unused]] volumetric_base::VolumetricState *state
) -> void
{
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinningBase::perform_blendshape_skinning(
    [[maybe_unused]] volumetric_base::VolumetricState *state,
    [[maybe_unused]] volumetric_base::SkinnableMesh &skel,
    [[maybe_unused]] volumetric_base::SkinnableMesh &musc,
    [[maybe_unused]] volumetric_base::SkinnableMesh &skin) -> void {
    std::cout << "Blendshape Skinning is not implemented for this method" << std::endl;
    perform_skinning(state,skel,musc,skin);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
