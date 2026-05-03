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

#include "Joint.h"

#include <pmp/types.h>

#include <utility>

//== IMPLEMENTATION ===========================================================
using namespace pmp;

namespace pmp_character
{
    Joint::Joint(std::string name, const pmp::Mat4<pmp::Scalar> &local) :
        name_(std::move(name)), parent_(nullptr), local_initial_(local), global_initial_(mat4::identity()), local_(local),
        global_(mat4::identity()), transformation_(mat4::identity()), is_effector_(false), bind_pose_local(local),
        bind_pose_global(mat4::identity())
    {
    }

    //-----------------------------------------------------------------------------

    Joint::~Joint()
    {
    }

    //-----------------------------------------------------------------------------

    void Joint::initialize_kinematics()
    {
        /**
         * @todo Initialize forward kinematic.
         *
         * - The initial local transformations are given (loaded from file)
         * - Calculate the initial global transformation matrix for this joint:
         *      `G[me] = G[parent] * L[me]`
         *   and store it in `global_` and `global_initial_`.
         *   This is the matrix that transforms this joint
         *   from the origin to its global position.
         *
         * Hints:
         * - The root joint does not have a parent, hence G[me] = L[me].
         *
         */

#ifndef STRIP_NOT_THIS_TIME_CODE

        // from local to global
        if (parent_)
        {
            global_ = parent_->global_ * local_;
        }
        else
        {
            global_ = local_;
        }

        // store initial global matrix
        global_initial_ = global_;
        bind_pose_global = global_;

#endif // STRIP_NOT_THIS_TIME_CODE

        for (auto joint : children_)
        {
            joint->initialize_kinematics();
        }
    }

    //-----------------------------------------------------------------------------
    ///
    ///- Calculate the current global transformation matrix for this joint:
    ///     `G[me] = G[parent] * L[me]`
    ///  and store it in the variable global_.
    ///- Compute the final skinning transformation matrix
    ///     `T[me] = `G[me] * G_initial[me]^-1`
    ///  and store it in `transformation_`
    void Joint::forward_kinematics()
    {
        /**
         * @todo Update forward kinematic.
         *
         * - Calculate the current global transformation matrix for this joint:
         *      `G[me] = G[parent] * L[me]`
         *   and store it in the variable global_.
         * - Compute the final skinning transformation matrix
         *      `T[me] = `G[me] * G_initial[me]^-1`
         *   and store it in `transformation_`.
         *
         * Hints:
         * - The root joint does not have a parent, hence `G[me] = L[me]`
         * - The code is very similar to the initialization of the forward kinematic
         *   in the `initialize_kinematics()` method
         * - the `inverse(...)` function computes the inverse of a matrix
         */

        if (parent_ )
        {
            global_ = parent_->global_ * local_;
        }
        else
        {
            global_ = local_;
        }
        // Global initial to bind pose
        transformation_ = bind_pose_global * inverse(global_initial_);
        // bind pose to final pose
        transformation_ = global_ * inverse(bind_pose_global) * transformation_;
        // recursively call function for all children
        for (auto joint : children_)
        {
            joint->forward_kinematics();
        }
    }

    //-----------------------------------------------------------------------------

    const Point Joint::get_global_translation() const
    {
        return Point(global_[12], global_[13], global_[14]);
    }

    //-----------------------------------------------------------------------------

    const pmp::Point Joint::get_global_initial_translation() const
    {
        return Point(global_initial_[12], global_initial_[13], global_initial_[14]);
    }

    //-----------------------------------------------------------------------------

    const pmp::Point Joint::get_local_initial_translation() const
    {
        return Point(local_initial_[12], local_initial_[13], local_initial_[14]);
    }

    //-----------------------------------------------------------------------------

    const pmp::Point Joint::get_local_translation() const
    {
        return Point(local_[12], local_[13], local_[14]);
    }

    //-----------------------------------------------------------------------------

    void Joint::rotate_joint(pmp::vec3 axis, float angle)
    {
        local_ = local_ * rotation_matrix(axis, angle);
    }

    //-----------------------------------------------------------------------------

    void Joint::set_rotation(const pmp::mat4 &rotation_matrix, bool perform_kinematics)
    {
        local_ = bind_pose_local * rotation_matrix;

        if (perform_kinematics)
        {
            forward_kinematics();
        }
    }

    //-----------------------------------------------------------------------------

    void Joint::set_local_translation(const pmp::vec3& translation)
    {
        local_[12] = translation[0];
        local_[13] = translation[1];
        local_[14] = translation[2];
    }

    //-----------------------------------------------------------------------------

} // namespace pmp_character
//=============================================================================
