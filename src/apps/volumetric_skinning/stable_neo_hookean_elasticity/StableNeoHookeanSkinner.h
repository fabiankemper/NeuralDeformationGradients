//
// Created by gerrit on 5/26/25.
//

#ifndef STABLENEOHOOKEANSKINNING_H
#define STABLENEOHOOKEANSKINNING_H


#include "StableNeoHookeanSimulation.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_character/volumetric_character.h"

namespace volumetric_skinning::neo_hookean {

    class NeoHookeanSkinner : public  SkinningBase {
    public:
        void perform_skinning(
            volumetric_base::VolumetricState* state,
            volumetric_base::SkinnableMesh& skel,
            volumetric_base::SkinnableMesh& musc,
            volumetric_base::SkinnableMesh& skin,
            bool converge
        ) override;
        auto updated_input_layers(volumetric_base::VolumetricState *state) -> void override;
        void updated_mean_layers(volumetric_base::VolumetricState* state) override;
        void updated_pose(volumetric_base::VolumetricState* state) override;
        void imgui_update(volumetric_base::VolumetricState *state) override;
        void doStep();

        bool draw_skel_weights = false;

        volumetric_base::LayeredMesh rest_pose_mesh;
        volumetric_base::LayeredMesh currently_animated_mesh_copy;
        volumetric_base::diffgeo::DifferentialGeometrySolver* _combinator = nullptr;
        StableNeoHookeanSkinning* _animator  = nullptr;
        std::vector<double> energies= {};
    };

} // nonlinear_skinning
// volumetric_skinning

#endif //STABLENEOHOOKEANSKINNING_H
