// =====================================================================================================================
//
// Created by Fabian Kemper on 2025-04-10.
//
// =====================================================================================================================

#include "volumetric_base/state/io.h"

#include <pmp/surface_mesh.h>
#include <pmp_character/fbxfile/fbx_file.h>

#include "volanim_utils/string_utils.h"
#include "volanim_utils/io/io_npz.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(io)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto state_read_highres_mesh(
    VolumetricState* state,
    const Layer layer,
    const Path& filename,
    const std::string& mesh_name
) -> void
{
    if (state == nullptr) { return; }

    const auto fbx_character = std::make_shared<pmp_character::Character>();
    auto fbx_file = pmp_character::FBX_file();

    // final read mesh from fbx
    pmp::SurfaceMesh mesh;

    // check if read was successful
    try {
        fbx_file.read(fbx_character.get(), filename);
        const auto mesh_node = fbx_character->get_mesh_by_name(mesh_name);
        mesh = mesh_node.mesh;
    } catch (const pmp_character::MeshNotFoundException& exception) {
        std::cerr << "[ERROR] Reading Highres meshes: Node " << mesh_name << " not found in FBX file.\n";
        return;
    }

    // check if loaded correctly
    if (mesh.n_vertices() == 0) {
        std::cerr << "[WARN] Reading High-res meshes: Node " << mesh_name << " was empty mesh.\n";
    }

    // store mesh in state
    state->set_highres_mesh(layer, mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

auto read_skinning_parameters_from_npz(VolumetricState* state, const Path& filename) -> void
{
    if (state == nullptr) { return; }

    volanim::utils::NPZFile file(filename, volanim::utils::ZipOpenMode::ReadOnly);

    auto read_layer = [&](const Layer& layer, const std::string& layer_name) -> void {
        constexpr size_t n_deps = ivec8::size();
        const size_t n_parameters = state->n_vertices();

        const auto npy_depends = file.get_entry<int>("vdepends_" + layer_name);
        const auto npy_weights = file.get_entry<float>("vweights_" + layer_name);

        if (npy_depends.shape[0] != n_parameters || npy_weights.shape[0] != n_parameters) {
            std::cerr << "Incorrect parameter count (vertex dim) for " + layer_name + ". Found " << npy_depends.shape[0] << " expected " << n_parameters << ".\n";
            return;
        }
        if (npy_depends.shape[1] != n_deps || npy_weights.shape[1] != n_deps) {
            std::cerr << "Incorrect parameter count (depends dim) for " + layer_name + ". Found " << npy_depends.shape[1] << " expected " << n_deps << ".\n";
            return;
        }

        SkinningParameters skinning_parameters;

        // copy depends
        skinning_parameters.resize(n_parameters);
        for (size_t idx = 0; idx < n_parameters; ++idx) {
            ivec8 depends;
            vec8 weights;
            for (size_t idx_dep = 0; idx_dep < n_deps; ++idx_dep) {
                depends[idx_dep] = npy_depends.data[idx * n_deps + idx_dep];
                weights[idx_dep] = npy_weights.data[idx * n_deps + idx_dep];
            }
            skinning_parameters[idx] = std::make_tuple(depends, weights);
        }

        state->set_skinning_parameters(layer, skinning_parameters);
    };

    read_layer(Layer::SKEL, "skel");
    read_layer(Layer::MUSC, "musc");
    read_layer(Layer::SKIN, "skin");
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_skinning_parameters_as_npz(VolumetricState* state, const Path& filename) -> void
{
    if (state == nullptr) { return; }

    volanim::utils::NPZFile file(filename, volanim::utils::ZipOpenMode::New);

    auto write_layer = [&](volanim::utils::NPZFile& npz_file, const SkinningParameters& skinning_parameters, const std::string& layer_name) -> void {
        constexpr size_t n_deps = ivec8::size();
        const auto n_parameters = skinning_parameters.size();

        std::vector<int> vdepends_data(n_parameters * n_deps);
        std::vector<float> vweights_data(n_parameters * n_deps);

        // copy values
        size_t offset = 0;
        for (auto& [vdepends, vweight] : skinning_parameters) {
            for (size_t index = 0; index < n_deps; ++index) {
                vdepends_data[offset] = vdepends[index];
                vweights_data[offset] = vweight[index];
                ++offset;
            }
        }

        npy::npy_data_ptr<int> npy_depends;
        npy_depends.data_ptr = vdepends_data.data();
        npy_depends.shape = { static_cast<npy::ndarray_len_t>(n_parameters), static_cast<npy::ndarray_len_t>(n_deps) };

        npy::npy_data_ptr<float> npy_weights;
        npy_weights.data_ptr = vweights_data.data();
        npy_weights.shape = { static_cast<npy::ndarray_len_t>(n_parameters), static_cast<npy::ndarray_len_t>(n_deps) };

        npz_file.add_entry<int>("vdepends_" + layer_name, npy_depends);
        npz_file.add_entry<float>("vweights_" + layer_name, npy_weights);
    };

    // every layer
    write_layer(file, state->get_skinning_parameters(Layer::SKEL), "skel");
    write_layer(file, state->get_skinning_parameters(Layer::MUSC), "musc");
    write_layer(file, state->get_skinning_parameters(Layer::SKIN), "skin");

    file.write();
}

// ---------------------------------------------------------------------------------------------------------------------

auto read_rig_from_file(VolumetricState* state, const Path& filename) -> void
{
    if (state == nullptr) { return; }
    auto& rig = state->rig;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file " << filename << " for reading.\n";
        return;
    }

    // clear current rig
    for (auto* joint : rig.joints_) {
        delete joint;
    }
    rig.joints_.clear();
    rig.root_ = nullptr;
    rig.selected_joint_ = -1;

    // read lines from file
    std::string line;
    std::vector<std::vector<std::string>> joint_tokens;

    while (std::getline(file, line)) {
        if (not line.empty() && line[0] != '#') {
            std::vector<std::string> tokens = volanim::utils::split(line, "\t");
            if (!tokens.empty() && tokens.size() == 19) {

                // read name
                std::string name = tokens[1];

                // read local matrix
                pmp::Mat4<float> local;
                for (auto idx = 0; idx < pmp::mat4::rows(); ++idx) {
                    for (auto jdx = 0; jdx < pmp::mat4::cols(); ++jdx) {
                        constexpr size_t matrix_offset = 3;
                        float value = std::stof(tokens[matrix_offset + idx * pmp::mat4::rows() + jdx]);
                        local(idx, jdx) = value;
                    }
                }

                auto* new_joint = new Joint(name, local);
                rig.joints_.emplace_back(new_joint);
                joint_tokens.emplace_back(tokens);
            } else {
                std::cerr << "Rig parser: Could not parse line " << line << "\n";
            }
        }
    }

    // parse parent information
    for (unsigned int idx = 0; idx < rig.joints_.size(); ++idx) {
        const auto& tokens = joint_tokens[idx];
        auto joint = rig.joints_[idx];
        size_t joint_id = std::stoi(tokens[0]);
        size_t parent_id = std::stoi(tokens[2]);
        if (joint_id < rig.joints_.size()) {
            joint->id = static_cast<int>(joint_id);
        }
        if (parent_id == static_cast<size_t>(-1)) {
            rig.root_ = joint;
        }
        if (parent_id < rig.joints_.size()) {
            joint->parent_ = rig.joints_[parent_id]; // set parent
            rig.joints_[parent_id]->children_.emplace_back(joint); // add to children of parent
        } else {
            joint->parent_ = nullptr;
        }
    }

    file.close();

    // initialize and update kinematic chain
    rig.init();
    rig.forward_kinematics();

    // update revision of state
    state->update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_rig_to_file(VolumetricState* state, const Path& filename) -> void
{
    if (state == nullptr) { return; }
    auto& rig = state->rig;

    std::ofstream skel_file(filename);

    if (!skel_file.is_open()) {
        std::cerr << "Could not open file " << filename << " for writing.\n";
        return;
    }

    skel_file << "#id\tname\tparent_id\tlocal_matrix (16 entries)\n";

    for (auto* joint : rig.joints_) {
        constexpr char separator = '\t';
        skel_file << joint->id << separator; // internal id
        skel_file << joint->name_ << separator; // name

        // parent joint
        if (joint->parent_ != nullptr) {
            skel_file << joint->parent_->id << separator; // parent joint
        } else {
            skel_file << "-1" << separator;
        }

        // use bindpose locals
        auto& local_mat = joint->local_initial_;
        for (auto idx = 0; idx < pmp::mat4::rows(); ++idx) {
            for (auto jdx = 0; jdx < pmp::mat4::cols(); ++jdx) {
                skel_file << local_mat(idx, jdx);
                if (not (idx == pmp::mat4::rows() - 1 && jdx == pmp::mat4::cols() - 1)) {
                    skel_file << separator;
                }
            }
        }
        skel_file << "\n";
    }

    // close file
    skel_file.close();
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(io)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
