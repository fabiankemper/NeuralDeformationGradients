// =====================================================================================================================

#pragma once
#ifndef PMP_EXT_PMP_MATRIX_EXT_H
#define PMP_EXT_PMP_MATRIX_EXT_H

// =====================================================================================================================

#include <pmp/mat_vec.h>
#include <pmp/types.h>

#include "pmp_ext/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

using Point = pmp::Point;
using Scalar = pmp::Scalar;

// =====================================================================================================================

// todo: check is this implemented by pmp?
auto mat3_to_eigen3f(const pmp::Mat3<Scalar>& matrix) -> Eigen::Matrix3f;

auto eigen3f_to_mat4(const Eigen::Matrix3f& matrix) -> pmp::Mat4<Scalar>;
// make matrix homogenous
auto mat3_to_mat4(const pmp::Mat3<Scalar>& matrix) -> pmp::Mat4<Scalar>;

auto mat4_translation_part(const pmp::Mat4<Scalar>& matrix) -> Point;

auto mat4_rotation_part(const pmp::Mat4<Scalar>& matrix) -> pmp::Mat3<Scalar>;

auto mat4_set_translation(const pmp::Mat4<Scalar>& matrix, const pmp::Point& point) -> pmp::Mat4<Scalar>;

auto mat4_set_translation_inplace(pmp::Mat4<Scalar>& matrix, const pmp::Point& point) -> void;

// conversion between rotation matrices and euler angles
auto mat4_to_euler_angles(const pmp::Mat4<Scalar>& matrix) -> pmp::vec3;
auto euler_angles_to_mat4(const pmp::vec3& euler_angles) -> pmp::Mat4<Scalar>;
auto mat3_to_euler_angles(const pmp::Mat3<Scalar>& matrix) -> pmp::vec3;
auto euler_angles_to_mat3(const pmp::vec3& euler_angles) -> pmp::Mat3<Scalar>;


// conversion between rodriguez angles and rotation matrices
auto rodrigues_to_mat4(const pmp::vec3& rodrigues) -> pmp::Mat4<Scalar>;
auto mat4_to_rodrigues(const pmp::Mat4<Scalar>& matrix) -> pmp::vec3;
auto rodrigues_to_mat3(const pmp::vec3& rodriguez) -> pmp::Mat3<Scalar>;
auto mat3_to_rodrigues(const pmp::Mat3<Scalar>& rotation_matrix) -> pmp::vec3;


// converting to eigen affine
auto mat4_to_affine(const pmp::Mat4<Scalar>& matrix) -> Eigen::Affine3f;
auto affine_to_mat4(const Eigen::Affine3f& affine) -> pmp::Mat4<Scalar>;


// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================

#endif //PMP_EXT_PMP_MATRIX_EXT_H

// =====================================================================================================================
