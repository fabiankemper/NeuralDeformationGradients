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
#include "Types.h"

namespace pmp_character
{
    /// get the first quaternion of a dual quaternion (the primal one)
    /// \sa get_second_quat
    inline const pmp::vec4 get_first_quat(vec8 &dq) { return pmp::vec4(dq[0], dq[1], dq[2], dq[3]); }

    //-----------------------------------------------------------------------------

    /// get the second quaternion of a dual quaternion (the dual one)
    /// \sa get_first_quat
    inline const pmp::vec4 get_second_quat(vec8 &dq) { return pmp::vec4(dq[4], dq[5], dq[6], dq[7]); }

    //-----------------------------------------------------------------------------

    /// convert a dual quaternion (represented as 8D vector) to a 4x4 matrix
    inline pmp::Mat4<pmp::Scalar> dualquat_to_mat4(const vec8 &dq)
    {
        float w = dq[0], x = dq[1], y = dq[2], z = dq[3];
        float t0 = dq[4], t1 = dq[5], t2 = dq[6], t3 = dq[7];

        pmp::Mat4<pmp::Scalar> m;

        m(0, 0) = w * w + x * x - y * y - z * z;
        m(0, 1) = 2 * x * y - 2 * w * z;
        m(0, 2) = 2 * x * z + 2 * w * y;
        m(0, 3) = -2 * t0 * x + 2 * w * t1 - 2 * t2 * z + 2 * y * t3;

        m(1, 0) = 2 * x * y + 2 * w * z;
        m(1, 1) = w * w + y * y - x * x - z * z;
        m(1, 2) = 2 * y * z - 2 * w * x;
        m(1, 3) = -2 * t0 * y + 2 * t1 * z - 2 * x * t3 + 2 * w * t2;

        m(2, 0) = 2 * x * z - 2 * w * y;
        m(2, 1) = 2 * y * z + 2 * w * x;
        m(2, 2) = w * w + z * z - x * x - y * y;
        m(2, 3) = -2 * t0 * z + 2 * x * t2 + 2 * w * t3 - 2 * t1 * y;

        m(3, 0) = 0;
        m(3, 1) = 0;
        m(3, 2) = 0;
        m(3, 3) = 1;

        return m;
    }

    //-----------------------------------------------------------------------------

    /// convert a 4x4 matrix (rotation and translation) to a
    /// dual quaternion (represented as 8D vector)
    inline vec8 mat4_to_dualquat(const pmp::Mat4<pmp::Scalar>& m)
    {
        vec8 dq;

        const float fourXSquaredMinus1 = m(0, 0) - m(1, 1) - m(2, 2);
        const float fourYSquaredMinus1 = m(1, 1) - m(0, 0) - m(2, 2);
        const float fourZSquaredMinus1 = m(2, 2) - m(0, 0) - m(1, 1);
        const float fourWSquaredMinus1 = m(0, 0) + m(1, 1) + m(2, 2);

        int biggestIndex = 0;
        float fourBiggestSquaredMinus1 = fourWSquaredMinus1;
        if (fourXSquaredMinus1 > fourBiggestSquaredMinus1)
        {
            fourBiggestSquaredMinus1 = fourXSquaredMinus1;
            biggestIndex = 1;
        }
        if (fourYSquaredMinus1 > fourBiggestSquaredMinus1)
        {
            fourBiggestSquaredMinus1 = fourYSquaredMinus1;
            biggestIndex = 2;
        }
        if (fourZSquaredMinus1 > fourBiggestSquaredMinus1)
        {
            fourBiggestSquaredMinus1 = fourZSquaredMinus1;
            biggestIndex = 3;
        }

        float biggestVal = sqrt(fourBiggestSquaredMinus1 + float(1)) * float(0.5);
        float mult = float(0.25) / biggestVal;

        switch (biggestIndex)
        {
        case 0:
            dq[0] = biggestVal;
            dq[1] = (m(2, 1) - m(1, 2)) * mult;
            dq[2] = (m(0, 2) - m(2, 0)) * mult;
            dq[3] = (m(1, 0) - m(0, 1)) * mult;
            break;
        case 1:
            dq[0] = (m(2, 1) - m(1, 2)) * mult;
            dq[1] = biggestVal;
            dq[2] = (m(1, 0) + m(0, 1)) * mult;
            dq[3] = (m(0, 2) + m(2, 0)) * mult;
            break;
        case 2:
            dq[0] = (m(0, 2) - m(2, 0)) * mult;
            dq[1] = (m(1, 0) + m(0, 1)) * mult;
            dq[2] = biggestVal;
            dq[3] = (m(2, 1) + m(1, 2)) * mult;
            break;
        case 3:
            dq[0] = (m(1, 0) - m(0, 1)) * mult;
            dq[1] = (m(0, 2) + m(2, 0)) * mult;
            dq[2] = (m(2, 1) + m(1, 2)) * mult;
            dq[3] = biggestVal;
            break;

        default:
            assert(false);
            break;
        }

        // normalize first quaternion
        float n = norm(get_first_quat(dq));
        dq[0] /= n;
        dq[1] /= n;
        dq[2] /= n;
        dq[3] /= n;

        dq[4] = -0.5f * (m(0, 3) * dq[1] + m(1, 3) * dq[2] + m(2, 3) * dq[3]);
        dq[5] = 0.5f * (m(0, 3) * dq[0] + m(1, 3) * dq[3] - m(2, 3) * dq[2]);
        dq[6] = 0.5f * (-m(0, 3) * dq[3] + m(1, 3) * dq[0] + m(2, 3) * dq[1]);
        dq[7] = 0.5f * (m(0, 3) * dq[2] - m(1, 3) * dq[1] + m(2, 3) * dq[0]);

        return dq;
    }
} // namespace pmp_character
//=============================================================================
