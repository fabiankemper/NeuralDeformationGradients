//
// Created by gerrit on 09.07.24.
//

#include "corotational_utility.h"

#include <pmp_character/DualQuaternion.h>

#include "volumetric_character/volumetric_character.h"


using namespace Eigen;
using namespace pmp;
using namespace pmp_character;
using namespace std;
using namespace volumetric_character;

using dualquat = Eigen::Vector<double,8>;

void setup_mesh_properties(volumetric_base::LayeredMesh &mesh, SurfaceMesh &propSource) {
    using namespace pmp;
    using namespace std;
    auto dependencies = propSource.get_vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
    auto dependencies_new = mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
    auto weights = propSource.get_vertex_property<pmp_character::vec8>(volumetric_base::VP_SKINNING_WEIGHTS);
    auto  weights_new = mesh.vertex_property<pmp_character::vec8>(volumetric_base::VP_SKINNING_WEIGHTS);

    for (Vertex v: mesh.vertices()) {
        dependencies_new[v] = dependencies[v];
        weights_new[v] = weights[v];
    }
}

void addMaps(map<int,double> &collection, map<int,double>& addMap,double weight) {
    for(auto [key,value] : addMap) {
        if(!collection.contains(key)) {
            collection[key]=0;
        }
        collection[key]+=weight*value;
    }
}
Quaterniond normalizeQuaternion(const Eigen::Quaterniond& q) {
    Quaterniond normalized = q;
    if (q.w() < 0) {
        normalized.coeffs() = -q.coeffs();
    }
    return normalized;
}


tuple<Quaterniond,Vector3d> blend_dual_quats(const vector<dualquat> & dualquats, const vector<double> & weights) {
    dualquat quatsum = dualquat::Zero();
    auto get_first_quat = [&](dualquat &quat)->Vector4d{return {quat[0], quat[1], quat[2], quat[3]};};
    for(unsigned int i=0; i< dualquats.size();i++) {
        dualquat temp = dualquats[i];
        double dotprod = get_first_quat(quatsum).dot( get_first_quat(temp)) ;
        dualquat add_val = temp * weights[i];
        quatsum += dotprod>0 ? add_val: -add_val ;
    }
    quatsum /= get_first_quat(quatsum).norm();

    auto transmatrix = static_cast<Matrix4d>(dualquat_to_mat4(quatsum));
    Matrix3d rota = transmatrix.block<3,3>(0,0);
    Vector3d translation = transmatrix.block<3,1>(0,3);
    return {Quaterniond(rota), translation};
}


tuple<Quaterniond,Vector3d> blendQuaternions(const std::vector<Eigen::Quaterniond>& quaternions, const std::vector<Eigen::Vector3d>& translations, const std::vector<double>& weights) {
    vector<dualquat> dualquats;
    for(unsigned int i=0; i <quaternions.size();i++) {
        Matrix4d affine_mat = Matrix4d::Zero();
        affine_mat.block<3,3>(0,0) = quaternions[i].matrix();
        affine_mat.block<3,1>(0,3) = translations[i];
        dualquat dq = mat4_to_dualquat(static_cast<mat4>(affine_mat));
        dualquats.emplace_back(dq);
    }
    return blend_dual_quats(dualquats, weights);
}

void CorotationUtility::ComputeRotationsPerVertex(
    const volumetric_base::LayeredMesh &mesh, const volumetric_character::VolumetricCharacter &character) {

    //sets up rotations and translations and stores them in one big matrix.
    //
    using namespace pmp;
    using namespace Eigen;
    using namespace std;

    character.state()->rig.forward_kinematics();
    std::vector<mat4> _skinning_matrices;

    _skinning_matrices.resize(character.state()->rig.joints_.size());
    for (unsigned int i = 0; i < character.state()->rig.joints_.size(); ++i)
    {
        _skinning_matrices[i] = character.state()->rig.joints_[i]->transformation_;
    }

    std::vector<Quaterniond> rotations(mesh.n_virtual_vertices());
    std::vector<Vector3d> translations(mesh.n_virtual_vertices());
    for(int i=0; i< mesh.n_virtual_vertices();i++) {
        map<int,double> dependencies_and_weights = skinning_weights[i];
        vector<dualquat> skinning_dualquats;
        vector<double> weights;
        for(auto [depd, weight] : dependencies_and_weights) {
            auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);

            skinning_dualquats.emplace_back(static_cast<dualquat>(pmp_character::mat4_to_dualquat(static_cast<mat4>(skinning_matrix))));
            weights.emplace_back(weight);
        }
        auto [r,t] = blend_dual_quats(skinning_dualquats,weights);
        rotations[i] = r;
        translations[i]  =t;
    }
    this->rotations = rotations;
    this->translations = translations;
}

tuple<Quaterniond,Vector3d> CorotationUtility::get_rotation_and_translation(int i1,int i2,int i3,int i4) const
{
    vector quaternions = {rotations[i1],rotations[i2],rotations[i3],rotations[i4] };
    vector weights = {0.25,0.25,0.25,0.25};
    vector translats  = {translations[i1], translations[i2], translations[i3], translations[i4]};
    return blendQuaternions(quaternions,translats,weights);
}

tuple<Matrix3d,Vector3d> CorotationUtility::get_rotation_and_translation(const std::vector<int>& indices) {
    vector<Quaterniond> quaternions;
    vector<double> weights;
    vector<Vector3d> translats;
    for(int index : indices) {
        quaternions.emplace_back(rotations[index]);
        translats.emplace_back(translations[index]);
        weights.emplace_back(1.0/static_cast<double>(indices.size()));
    }
    auto [quat, vec ] = blendQuaternions(quaternions,translats,weights);
    return {quat.matrix(), vec};
}

CorotationUtility::CorotationUtility(volumetric_base::LayeredMesh &mesh,
                                     volumetric_character::VolumetricCharacter& character,
                                     SurfaceMesh& propSource)
{
    setup_mesh_properties(mesh, propSource);
    cleanMesh(mesh);

    VertexProperty<vec8> weights = mesh.get_vertex_property<pmp_character::vec8>(
            volumetric_base::VP_SKINNING_WEIGHTS);
    VertexProperty<ivec8> dependencies = mesh.get_vertex_property<pmp_character::ivec8>(
            volumetric_base::VP_DEPENDS);

    int n_real_vertices = mesh.n_layer_vertices();
    skinning_weights.resize(n_real_vertices);

    // 3. Populate map directly
    for (volumetric_base::LayerVertex vv : mesh.getLayerVertices()) {
        int index = mesh.Index(vv);
        pmp::Vertex v = get<0>(vv); // The underlying PMP vertex

        std::map<int, double> weightmap;

        for(int i = 0; i < 8; ++i) {
            double w = weights[v][i];
            if(w > 1e-6) {
                int joint_id = dependencies[v][i];
                weightmap.emplace(joint_id, w);
            }
        }
        skinning_weights[index] = weightmap;
    }


    this->ComputeRotationsPerVertex(mesh, character);
}

std::vector<std::map<int, double>> CorotationUtility::get_skinning_weights() const {
    return this->skinning_weights;
}
