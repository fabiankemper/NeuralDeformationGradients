// =====================================================================================================================
// Created by fabian on 9/11/24.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VOL_BASE_ANIMATION_H
#define VOL_BASE_ANIMATION_H

// =====================================================================================================================

#include "volanim_utils/defs.h"
#include "volumetric_base/aliases.h"

#include <vector>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(animation)

// =====================================================================================================================

class Animation {

public:
    Animation() = default;
    ~Animation() = default;

    // [deprecated] load animation from single file
    auto from_numpy(const Path& filename) -> void;
    // load animation from numpyz file
    auto from_numpyz(
        const Path& filename,
        const std::string& pose_entry = "poses",
        const std::string& trans_entry = "trans",
        const std::string& framerate_entry = "framerate"
    ) -> void;

    // load animation from csv file
    auto from_csv(const Path& filename) -> void;

    // load animation from pose-file
    auto from_posefile(const Path& filename) -> void;

    // current framerate
    [[nodiscard]]
    auto framerate() const -> double;

    [[nodiscard]]
    auto current_frame() const -> long;

    [[nodiscard]]
    auto total_frames() const -> long;

    auto set_frame(long frame) -> void;
    auto step_frames(long frames) -> void;

    [[nodiscard]]
    auto current_pose() const -> std::vector<float>;

    [[nodiscard]]
    auto current_trans() const -> std::array<float, 3>;

    auto read(std::filesystem::path filename) -> void;

protected:
    // current frame positions
    long _current_frame = 0;

    // total frames for this animation
    long _total_frames = 0;

    // pose parameter count for animation
    long _num_pose_parameters = 180; // todo: define magic number in another file

    // trans parameter count
    long _num_trans_parameters = 3;

    // frames per second for this animatio (if available)
    float _framerate = 60.0f;

    // frame buffer object -> [frame][pose parameter]
    std::vector<float> _poses_buffer;

    // trans buffer object [frame][trans parameter]
    std::vector<float> _trans_buffer;

};


// =====================================================================================================================

NAMESPACE_END(animation)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOL_BASE_ANIMATION_H

// =====================================================================================================================
