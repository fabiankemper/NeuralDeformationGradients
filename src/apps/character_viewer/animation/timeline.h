// =====================================================================================================================

#pragma once
#ifndef TIMELINE_H
#define TIMELINE_H

// =====================================================================================================================

#include "volanim_utils/defs.h"

#include <pmp/stop_watch.h>

#include "volumetric_base/aliases.h"
#include "volumetric_base/animation.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)
NAMESPACE_BEGIN(animation)

// =====================================================================================================================

using Animation = volumetric_base::animation::Animation;
using StopWatch = pmp::StopWatch;
using Path = volumetric_base::Path;

// =====================================================================================================================

class Timeline
{
public:
    Timeline();
    virtual ~Timeline();

    // import from converted numpy file
    auto from_numpy(const Path& filename) -> void;
    auto from_numpyz(const Path& filename) -> void;
    auto from_csv(const Path& filename) -> void;
    auto from_posefile(const Path& filename) -> void;

    // draw timeline
    auto process_imgui() -> void;
    auto check_key_press() -> void;

    // new animation parameters are available
    [[nodiscard]]
    auto is_dirty() const -> bool { return _dirty.load(); }
    auto clear_dirty() -> void { _dirty.store(false); }

    // get new pose parameters
    [[nodiscard]]
    auto current_pose() const -> std::vector<float>;

    // get new translation parameters
    [[nodiscard]]
    auto current_trans() const -> std::array<float, 3>;

    // get current frame
    [[nodiscard]]
    auto current_frame() const -> int;

    // new frame selectes
    auto select_frame(long index) -> void;

    // step frame
    auto step_frames(long frames) -> void;

    // is currently playing?
    [[nodiscard]]
    auto playing() const -> bool { return _playback_speed != 0.0F; }
    auto set_playing(bool playing) -> void;

    // playback speed
    // 0 = paused, 1.0 normal, -1.0 reverse
    [[nodiscard]]
    auto playback_speed() const -> float { return _playback_speed; }
    auto set_playback_speed(float playback_speed) -> void;

    // current fps from file
    [[nodiscard]]
    auto fps() const -> float { return static_cast<float>(_animation.framerate()); }

    // tick clock and return if new frame is available
    auto tick() -> bool;
protected:
    // animation data
    Animation _animation;

    // playing
    float _playback_speed = 0.0F;

    // for playing timeline
    StopWatch _timer;
    double _elapsed_time = 0.0;

    // for draw call: new frame availabke
    std::atomic<bool> _dirty = true;

    // helper for dpi scaling
    [[nodiscard]]
    static auto get_dpi_scale() -> float;

    // zoom level (0.0 fully out, 1.0 fully in)
    float _zoom_level = 0.6f;

    const float MIN_PX_PER_FRAME = 0.05F;
    const float MAX_PX_PER_FRAME = 16.0F;

    // internal state for every frame
    float _pixels_per_frame = 2.0f;

    // follow the playhead?
    bool _follow_playhead = true; // toggle for auto-scrolling

    // helper
    auto zoom_to_pixels(float zoom_factor) const -> float;
    auto pixels_to_zoom(float pixels) const -> float;
};

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(animation)
NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //TIMELINE_H

// =====================================================================================================================
