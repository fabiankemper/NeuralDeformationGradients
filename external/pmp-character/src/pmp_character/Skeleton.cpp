//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================

//== INCLUDES =================================================================

#include "Skeleton.h"

#include <cfloat>

using namespace pmp;
//== IMPLEMENTATION ===========================================================

namespace pmp_character
{
    Skeleton::Skeleton() : selected_joint_(-1), size_(0), center_()
    {
    }

    //-------------------------------- ---------------------------------------------
    Skeleton::Skeleton(const Skeleton& other) : root_(nullptr), selected_joint_(-1), size_(other.size_), center_(other.center_)
    {
        // Create the joints themselves
        joints_.resize(other.joints_.size());

        for (const auto other_joint : other.joints_) {
            const auto new_joint = new Joint(std::string(other_joint->name_), pmp::mat4(other_joint->local_));
            new_joint->local_initial_ = pmp::mat4(other_joint->local_initial_);
            new_joint->global_ = pmp::mat4(other_joint->global_);
            new_joint->global_initial_ = pmp::mat4(other_joint->global_initial_);
            new_joint->bind_pose_local = pmp::mat4(other_joint->bind_pose_local);
            new_joint->bind_pose_global = pmp::mat4(other_joint->bind_pose_global);
            new_joint->inverse_bind_matrix_ = pmp::mat4(other_joint->inverse_bind_matrix_); // todo: this is unused?
            new_joint->id = other_joint->id;
            new_joint->parent_ = nullptr;
            joints_[new_joint->id] = new_joint;
        }

        // Fix hierarchy
        for (const auto other_joint : other.joints_)
        {
            auto joint_id = other_joint->id;
            // set parent
            if (other_joint->parent_ != nullptr) {
                auto other_parent = other_joint->parent_;
                joints_[joint_id]->parent_ = joints_[other_parent->id];
            }

            for (const auto* child : other_joint->children_)
            {
                auto child_id = child->id;
                joints_[joint_id]->children_.emplace_back(joints_[child_id]);
            }
        }

        // set root
        if (other.root_ != nullptr) {
            root_ = joints_[other.root_->id];
        }

        forward_kinematics();
    }

    // copy-assignment (copy and swap)
    // Skeleton& Skeleton::operator=(const Skeleton& other)
    // {
    //     if (this == &other) {
    //         return *this;
    //     }
    //     Skeleton tmp(other);
    //     swap(*this, tmp);
    //     return *this;
    // }
    Skeleton& Skeleton::operator=(const Skeleton& other) = default;

    Skeleton::~Skeleton() = default;
    // {
    //     clear();
    // }

    //-----------------------------------------------------------------------------

    void Skeleton::init()
    {
        root_->initialize_kinematics();

        update();
    }

    //-----------------------------------------------------------------------------

    void Skeleton::forward_kinematics() const
    {
        if (root_ == nullptr)
            return;
        // trigger recursive traversal of joint hierarchy
        root_->forward_kinematics();
    }

    //-----------------------------------------------------------------------------

    Joint *Skeleton::get_joint(unsigned int idx) const
    {
        assert(idx < joints_.size());
        return joints_[idx];
    }

    //-----------------------------------------------------------------------------

    Joint *Skeleton::get_joint(const std::string &joint_name) const
    {
        for (auto joint : joints_)
        {
            if (joint->name_ == joint_name)
            {
                return joint;
            }
        }
        return nullptr;
    }

    //-----------------------------------------------------------------------------

    int Skeleton::get_joint_idx(const char *joint_name) const
    {
        for (size_t index = 0; index < joints_.size(); ++index)
        {
            Joint* joint = joints_[index];
            if (joint->name_ == joint_name)
            {
                return static_cast<int>(index);
            }
        }
        return -1;
    }

    //-----------------------------------------------------------------------------

    void Skeleton::select_joint(int j)
    {
        selected_joint_ = j;

        if (selected_joint_ < 0) {
            selected_joint_ = joints_.size() - 1;
        }
        else if (selected_joint_ >= (int)joints_.size()) {
            selected_joint_ = 0;
        }
    }

    //-----------------------------------------------------------------------------

    void Skeleton::rotate_joint(unsigned int joint_idx, Point axis, double angle)
    {
        if (joint_idx < joints_.size())
        {
            Joint *joint = joints_[joint_idx];
            joint->local_ = joint->local_ * rotation_matrix(axis, pmp::Scalar(angle));
            update();
        }
    }

    //-----------------------------------------------------------------------------

    int Skeleton::pick_joint(const vec2 &coord2d, const M4x4& mvp)
    {
        float closest_dist = FLT_MAX;

        int joint = -1;
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            const Point p = joints_[i]->get_global_translation();
            pmp::Vector<Scalar,4> ndc = mvp * pmp::Vector<Scalar,4>(p, 1.0f);
            ndc /= ndc[3];

            const float d = distance(vec2(ndc[0], ndc[1]), coord2d);
            if (d < closest_dist)
            {
                closest_dist = d;
                joint = (int)i;
            }
        }

        return joint;
    }

    //-----------------------------------------------------------------------------

    void Skeleton::reset()
    {
        // reset local transformations
        for (Joint *joint : joints_)
        {
            joint->local_ = joint->bind_pose_local;
            joint->is_effector_ = false;
            selected_joint_ = 0;
        }

        // perform forward kinematics
        update();
    }

    //-----------------------------------------------------------------------------

    void Skeleton::reset_locals()
    {
        // reset local transformations
        for (Joint *joint : joints_)
        {
            joint->local_ = joint->bind_pose_local;
        }

        // perform forward kinematics
        update();
    }
    void Skeleton::update_joint_positions(std::vector<pmp::Point> newPositions)
    {
        reset();
        forward_kinematics();

        size_t num_joints = joints_.size();

        if (num_joints != newPositions.size())
        {
            fprintf(stderr, "[ERROR] set_new_joint_positions: num joints does not match num new joints");
            fprintf(stderr, " (%zu vs %zu)\n", num_joints, newPositions.size());
            return;
        }

        for (size_t i = 0; i < num_joints; ++i)
        {
            Joint* j = joints_[i];
            const pmp::Point& p = newPositions[i];
            // set new joint position as translation for this joint
            j->global_initial_(0, 3) = p[0];
            j->global_initial_(1, 3) = p[1];
            j->global_initial_(2, 3) = p[2];
            j->global_ = j->global_initial_;
            j->bind_pose_global = j->global_initial_;
        }

        for (size_t i = 0; i < num_joints; ++i)
        {
            Joint* j = joints_[i];

            pmp::Point from_parent;
            if (j->parent_)
            {
                from_parent = j->get_global_translation() - j->parent_->get_global_translation();
                // transform offset vector from 'parent' to j into the coordinate system of 'parent'
                from_parent = transpose(linear_part(j->parent_->global_)) * from_parent;
            }
            else
            {
                from_parent = j->get_global_translation();
            }

            j->local_initial_(0, 3) = from_parent[0];
            j->local_initial_(1, 3) = from_parent[1];
            j->local_initial_(2, 3) = from_parent[2];

            // reset pose
            j->local_ = j->local_initial_;
            j->bind_pose_local = j->local_initial_;
        }
        reset();
        forward_kinematics();
        init();
    }

    void Skeleton::set_current_pose_as_bindpose(bool perform_kinematics)
    {
        if(perform_kinematics)
        {
            forward_kinematics();
        }
        for(auto joint : joints_)
        {
            joint->bind_pose_global = joint->global_;
            joint->bind_pose_local = joint->local_;
        }
    }

    void Skeleton::clear_bindpose()
    {
        for(auto joint : joints_)
        {
            joint->bind_pose_global = joint->global_initial_;
            joint->bind_pose_local = joint->local_initial_;
        }
        forward_kinematics();
    }

    auto Skeleton::clear() noexcept -> void
    {
        for (auto* joint : joints_) {
            delete joint;
        }
        joints_.clear();
        root_ = nullptr;
        selected_joint_ = -1;
    }

    auto swap(Skeleton& a, Skeleton& b) noexcept -> void
    {
        std::swap(a.root_, b.root_);
        std::swap(a.joints_, b.joints_);
        std::swap(a.selected_joint_, b.selected_joint_);
        std::swap(a.size_, b.size_);
        std::swap(a.center_, b.center_);
    }

    //-----------------------------------------------------------------------------

} // namespace pmp_character
//=============================================================================
