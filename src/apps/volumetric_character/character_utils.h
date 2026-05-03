// =====================================================================================================================

#ifndef VOLUMETRIC_ANIMATION_CHARACTER_UTILS_H
#define VOLUMETRIC_ANIMATION_CHARACTER_UTILS_H

// =====================================================================================================================

#include "volanim_utils/defs.h"

#include <pmp/surface_mesh.h>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)
NAMESPACE_BEGIN(utils)

// =====================================================================================================================


// copy property from one mesh to another.
template<typename T>
auto copy_pmp_property(pmp::Property<T> source, pmp::Property<T> target, size_t length) -> void {
    for (size_t index = 0; index < length; ++index) {
        target[index] = source[index];
    }
}

template<typename T>
auto clamp(T value, T min, T max) -> T {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

// =====================================================================================================================

NAMESPACE_END(utils)
NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //VOLUMETRIC_ANIMATION_CHARACTER_UTILS_H

// =====================================================================================================================
