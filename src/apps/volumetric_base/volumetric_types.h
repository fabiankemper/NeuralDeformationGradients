// =====================================================================================================================
//
// Created by Fabian Kemper on 2024-09-02.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_TYPES_H
#define VOLUMETRIC_TYPES_H

// =====================================================================================================================

#include "pmp_ext/defs.h"

#include <array>
#include <iostream>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

enum struct Layer : char {
    SKEL = 0,
    MUSC = 1,
    SKIN = 2,
};

// helper for range based loops
constexpr std::array<Layer, 3> ALL_LAYERS = {Layer::SKEL, Layer::MUSC, Layer::SKIN};

enum struct Gender
{
    FEMALE,
    MALE,
};

// helper for printing
inline std::ostream& operator<<(std::ostream& os, const Gender& gender)
{
    switch (gender) {
        case Gender::MALE:
            os << "Gender::MALE"; break;
        case Gender::FEMALE:
            os << "Gender::FEMALE"; break;
        default:
            os << "[Unknown Gender]"; break;
    }
    return os;
}

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_TYPES_H

// =====================================================================================================================
