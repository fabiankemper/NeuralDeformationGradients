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
#include <pmp/types.h>

#include "Types.h"

namespace pmp_character
{
#include <vector>

    //== CLASS DEFINITION =========================================================

    /** Class for representing joints in a skeleton hierarchy.
        The Joint stores a set of local and global matrices that define
        the posture of the skeleton. These matrices are stored in initial state
        and in the current animated state. It also stores the transformation
        from initial to animated global coordiantes, which are used for skinning.
        For inverse kinematics computations, each joint stores whether it is an
        effector and what its target position is.
     */

    class Joint
    {
    public:
        /// constructor
        Joint(std::string name, const pmp::Mat4<pmp::Scalar> &local);

        /// destructor
        virtual ~Joint();

        /// recursively initialize global matrices from local matrices
        void initialize_kinematics();

        /// recursively update global matrices through forward kinematics
        void forward_kinematics();

        /// return current global translation of this joint
        /// (i.e., the origin of the joint's coordinate system)
        const pmp::Point get_global_translation() const;

        const pmp::Point get_global_initial_translation() const;

        const pmp::Point get_local_initial_translation() const;

        const pmp::Point get_local_translation() const;


        /// rotate the joint with index joint_idx around axis and angle
        void rotate_joint(pmp::vec3 axis, float angle);

        /// set rotation matrix to given value
        void set_rotation(const pmp::mat4& rotation_matrix, bool perform_kinematics = true);

        /// set local transformation translation
        void set_local_translation(const pmp::vec3& translation);

        /// is root joint (parent==NULL)?
        bool is_root() const { return parent_ == nullptr; }

    public:
        /// name of this joint
        std::string name_;

        int id;

        /// parent of this joint (NULL, if root)
        Joint *parent_;

        /// the children of this joint
        std::vector<Joint *> children_;

        /// the local transformation matrix in initial pose.
        /// is loaded from file, never changed, and hence constant
        pmp::Mat4<pmp::Scalar> local_initial_;

        /// the global transformation matrix in initial pose.
        /// is computed through the init() method.
        pmp::Mat4<pmp::Scalar> global_initial_;

        /// Global transformation matrix that can be set via the @link set_bind_pose method in the skeleton
        M4x4 bind_pose_global;

        /// Global transformation matrix that can be set via the @link set_bind_pose method in the skeleton
        M4x4 bind_pose_local;

        /// the current local transformation matrix.
        /// maps from parents coordinate system to this coordinate system.
        /// is set by user or through playing an animation.
        pmp::Mat4<pmp::Scalar> local_;

        /// the current global transformation matrix.
        /// maps from the root coordinate system to this coordinate system.
        /// is computed form current local transformation through forward_kinematics()
        pmp::Mat4<pmp::Scalar> global_;

        /// the transformation mapping the initial global coordinate system to the
        /// current global coordinate system (global_ * inverse(global_initial)).
        /// this matrix is the given to the skinning method.
        pmp::Mat4<pmp::Scalar> transformation_;

        ///
        pmp::Mat4<pmp::Scalar> inverse_bind_matrix_;

        /// is joint an effector for inverse kinematics?
        bool is_effector_;

        /// target position for inverse kinematics
        pmp::vec3 target_;
    };
} // namespace pmp_character
//============================================================================
