#pragma once
#ifndef PROJECTIVE_SKINNING_H
#define PROJECTIVE_SKINNING_H


#include "volumetric_base/layered_mesh.h"
#include "volumetric_character/volumetric_character.h"

namespace projective_skinning {


using Matrix=Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> ;
class ProjectiveSkinner{

public:
    virtual ~ProjectiveSkinner() = default;

    explicit ProjectiveSkinner(volumetric_base::LayeredMesh mesh );

    void update_skinning_with_dualquats(const volumetric_base::LayeredMesh &mesh, const volumetric_character::VolumetricState &state);

    void do_global_step(const volumetric_base::LayeredMesh & deformable_mesh);
    void do_local_step(const volumetric_base::LayeredMesh & deformable_mesh);
    void do_full_steps(const volumetric_base::LayeredMesh & deformable_mesh, int num_steps);

    volumetric_base::LayeredMesh undeformed_mesh;



protected:
    //rotation fitting stuff
    std::vector<float> rotations;
    std::vector<float> deformation_grads;
    int num_groups;
    //precomputed stuff
    Matrix C;
    Matrix results;
    Matrix defgrads;
    Matrix rhs;
    Matrix righthandside;
    std::vector<volumetric_base::Prism> cells;
    Eigen::SparseMatrix<double> gradient_matrix;
    Eigen::SparseMatrix<double> lefthandside;
    Eigen::SparseMatrix<double> divergence;
    PrecomputeSolver precompute_solver;
    bool rotations_dirty=true;
}
;

class ProjectiveSkinnerInterface : public volumetric_skinning::SkinningBase{
    public:
        void perform_skinning(
            volumetric_base::VolumetricState* state,
            volumetric_base::SkinnableMesh& skel,
            volumetric_base::SkinnableMesh& musc,
            volumetric_base::SkinnableMesh& skin,
            bool converge
        ) override;
        void updated_input_layers(volumetric_base::VolumetricState* state) override;
        void updated_mean_layers(volumetric_base::VolumetricState* state) override;
        void updated_pose(volumetric_base::VolumetricState* state) override;
        void imgui_update(volumetric_base::VolumetricState *state) override;
        void doStep() const;

        volumetric_base::LayeredMesh rest_pose_mesh;
        volumetric_base::LayeredMesh _combined_copy;
        volumetric_base::diffgeo::DifferentialGeometrySolver* _diffgeo_solver = nullptr;
        ProjectiveSkinner* _animator  = nullptr;
};


} // namespace projective_skinning

#endif //PROJECTIVE_SKINNING_H
