//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================
#pragma once
//=============================================================================
#include <pmp/mat_vec.h>

#include "Joint.h"
#include "Types.h"
namespace pmp_character
{
#include "Joint.h"

#include <vector>

    //== CLASS DEFINITION =========================================================

    /** Skeleton class for controlling a virtual character.
        It stores a vector of joints, which represent a tree through the joints'
        parent and children pointers. After setting local transformations per
        joint, the skeleton can compute the global transformations through its
        forward_kinematics() method.
        \sa Joint
    */
    class Skeleton
    {
    public:
        /// constructor
        Skeleton();
        Skeleton(const Skeleton &other);
        // remove the copy assignment operator, after copy assignment the old object is remove
        // due to a shallow copy, this leads to deletion of the associated joints, which are free'd.
        Skeleton& operator=(const Skeleton& other);
        // only allow move construction
        Skeleton(Skeleton&&) noexcept = default;

        /// destructor
        virtual ~Skeleton();

        /// initialize joints
        void init();

        /// update forward kinematic with local axes matrices (see class Joint).
        void forward_kinematics() const;
        /// update skeleton (calls forward_kinematics()). will be overloaded by
        /// Character class.
        virtual void update() { forward_kinematics(); }

        /// size of the model
        [[nodiscard]]
        float get_size() const { return size_; }
        /// center of the model
        [[nodiscard]]
        const pmp::Point &get_center() const { return center_; }

        /// get joint from its index
        [[nodiscard]]
        Joint *get_joint(unsigned int idx) const;
        /// get joint from its joint_name
        [[nodiscard]]
        Joint *get_joint(const std::string &joint_name) const;
        /// get joint index from its name
        int get_joint_idx(const char *joint_name) const;

        /// pick joint closest to 2D point p (mouse coordinate).
        /// returns the index of the selected joint.
        int pick_joint(const pmp::vec2 &coord2d, const M4x4& mvp);

        /// select a joint
        virtual void select_joint(int j);
        /// get selected joint index
        [[nodiscard]]
        int selected_joint() const { return selected_joint_; }

        /// rotate joint around axis and angle
        void rotate_joint(unsigned int joint, pmp::Point axis, double angle);

        /// reset skeleton to initial posture
        void reset();

        /// reset skeleton to initial posture but leave effectors unchanged
        void reset_locals();

        /// Updates all joint positions to the new Positions provided
        void update_joint_positions(std::vector<pmp::Point> newPositions);

        ///
        void set_current_pose_as_bindpose(bool perform_kinematics = true);

        // clear bindpose
        void clear_bindpose();

        /// The root joint
        Joint *root_ = nullptr;

        /// joint pointers
        std::vector<Joint *> joints_;

        /// selected joint (for user interaction and weight visualization)
        int selected_joint_ = -1;

        /// size of the model
        pmp::Scalar size_;
        /// center of the model
        pmp::Point center_;

    protected:
        friend auto swap(Skeleton& a, Skeleton& b) noexcept -> void;
        auto clear() noexcept -> void;

    };
} // namespace pmp_character
//=============================================================================
