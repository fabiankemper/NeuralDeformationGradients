// =====================================================================================================================

#include "timeline.h"

#include <imgui.h>
#include <imgui_internal.h>

#include "character_viewer/constants.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)
NAMESPACE_BEGIN(animation)

// =====================================================================================================================

// local helpers
// Helper to convert User Zoom (0-1) to internal PxPerFrame (Logarithmic)
auto Timeline::zoom_to_pixels(const float zoom_factor) const -> float {
    // formula: min * (max/min)^zoom
    return MIN_PX_PER_FRAME * std::pow(MAX_PX_PER_FRAME / MIN_PX_PER_FRAME, zoom_factor);
}

// Helper to convert internal PxPerFrame to User Zoom (0-1)
auto Timeline::pixels_to_zoom(const float px_per_frame) const -> float {
    // formula: log(px / min) / log(max / min)
    return std::log(px_per_frame / MIN_PX_PER_FRAME) / std::log(MAX_PX_PER_FRAME / MIN_PX_PER_FRAME);
}

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

Timeline::Timeline() = default;

// ---------------------------------------------------------------------------------------------------------------------

Timeline::~Timeline() = default;

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::from_numpy(const Path& filename) -> void
{
    _animation.from_numpy(filename);
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::from_numpyz(const Path& filename) -> void
{
    _animation.from_numpyz(filename);
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::from_csv(const Path& filename) -> void
{
    _animation.from_csv(filename);
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::from_posefile(const Path& filename) -> void
{
    _animation.from_posefile(filename);
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::process_imgui() -> void
{
    check_key_press();

    const int total_frames   = static_cast<int>(_animation.total_frames());
    const int current_frame  = static_cast<int>(_animation.current_frame());
    _pixels_per_frame = zoom_to_pixels(_zoom_level);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(3.0F, 0));
    ImGui::Begin("Timeline", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // --- Scrolling Region ---
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(1.0F, 0));
    ImGui::BeginChild("TimelineScrollRegion", ImVec2(0, 50.0F), 1, ImGuiWindowFlags_AlwaysHorizontalScrollbar);

    if (ImGui::IsWindowHovered() && ImGui::GetIO().MouseWheel != 0.0F) {
        const float mouse_x_in_canvas  = ImGui::GetMousePos().x - ImGui::GetCursorScreenPos().x + ImGui::GetScrollX();
        const float frame_under_mouse  = mouse_x_in_canvas / _pixels_per_frame;

        // Adjust the abstract zoom level (e.g., 5% change per notch)
        _zoom_level += ImGui::GetIO().MouseWheel * 0.05f;
        _zoom_level = std::clamp(_zoom_level, 0.0f, 1.0f);

        // Re-calculate pixels after zoom change
        _pixels_per_frame = zoom_to_pixels(_zoom_level);

        // Fix scroll position so the frame stays under the mouse
        ImGui::SetScrollX(frame_under_mouse * _pixels_per_frame - (ImGui::GetMousePos().x - ImGui::GetCursorScreenPos().x));
    }

    const float timeline_width = static_cast<float>(total_frames) * _pixels_per_frame + 50.0F;
    const ImVec2 canvas_pos    = ImGui::GetCursorScreenPos();
    const ImVec2 canvas_size   = ImGui::GetContentRegionAvail();
    const float scroll_x       = ImGui::GetScrollX();

    ImGui::SetCursorPos(ImVec2(0, 0));
    ImGui::InvisibleButton("##TimelineTouchLayer", ImVec2(timeline_width, canvas_size.y));

    if (ImGui::IsItemActive()) {
        int new_frame = static_cast<int>((ImGui::GetMousePos().x - canvas_pos.x) / _pixels_per_frame);
        new_frame = std::clamp(new_frame, 0, total_frames - 1);
        if (new_frame != current_frame)
            select_frame(new_frame);
    }

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    int vis_start = std::max(0, static_cast<int>(scroll_x / _pixels_per_frame));
    int vis_end   = std::min(total_frames, static_cast<int>((scroll_x + canvas_size.x) / _pixels_per_frame) + 1);

    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + timeline_width, canvas_pos.y + canvas_size.y), IM_COL32(222, 222, 222, 255));

    int stride = 1;
    int multiplier = 1;

    constexpr float min_pixels_between_ticks = 6.0f;

    // This logic cycles through 1, 2, 5, 10, 20, 50, 100...
    while (_pixels_per_frame * stride < min_pixels_between_ticks) {
        if (multiplier == 1) {
            multiplier = 2;
            stride = stride / 1 * 2;
        } else if (multiplier == 2) {
            multiplier = 5;
            stride = stride / 2 * 5;
        } else {
            multiplier = 1;
            stride = stride / 5 * 10;
        }
    }

    // Define major ticks based on a clean base-10 multiple
    // If stride is 1, 2, or 5, major is 10.
    // If stride is 10, 20, 50, major is 100, etc.
    int major_stride = 10;
    while (major_stride <= stride) {
        major_stride *= 10;
    }
    // Optional: adjust major_stride to be specifically 10x the current visual stride
    major_stride = stride * 10;

    for (int i = vis_start; i <= vis_end; ++i) {
        if (i % stride != 0 && i != total_frames) {
            continue;
        }

        const float x        = canvas_pos.x + static_cast<float>(i) * _pixels_per_frame;
        // Major ticks land on clean decimal boundaries (10, 100, 1000...)
        const bool is_major = (i % major_stride == 0) || i == 0;
        const float h        = canvas_size.y * (is_major ? 0.9F : 0.5F);
        const ImU32 col      = is_major ? IM_COL32(100, 100, 100, 255) : IM_COL32(140, 140, 140, 100);

        draw_list->AddLine(ImVec2(x, canvas_pos.y + canvas_size.y - h), ImVec2(x, canvas_pos.y + canvas_size.y), col, 1.0F);

        if (is_major) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", i);
            draw_list->AddText(ImVec2(x + 2, canvas_pos.y), IM_COL32(75, 75, 75, 255), buf);
        }
    }

    // Playhead
    const float cursor_x = canvas_pos.x + static_cast<float>(current_frame) * _pixels_per_frame;
    draw_list->AddLine(ImVec2(cursor_x, canvas_pos.y), ImVec2(cursor_x, canvas_pos.y + canvas_size.y), IM_COL32(255, 50, 50, 255), 2.0F);
    draw_list->AddTriangleFilled(
        ImVec2(cursor_x - 5.0F, canvas_pos.y),
        ImVec2(cursor_x + 5.0F, canvas_pos.y),
        ImVec2(cursor_x,        canvas_pos.y + 10.0F),
        IM_COL32(255, 50, 50, 255)
    );

    if (playing() && _follow_playhead) {
        const float rel_x = static_cast<float>(current_frame) * _pixels_per_frame - scroll_x;
        if (rel_x < 0.0F || rel_x > canvas_size.x) {
            ImGui::SetScrollX(static_cast<float>(current_frame) * _pixels_per_frame - canvas_size.x * 0.5F);
        }
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(); // Pop padding for the Child


    // --- Controls ---
    if (playing()) {
        if (ImGui::Button("pause##timeline_pause", ImVec2(48, 0))) {
            set_playing(false);
        }
    } else {
        if (ImGui::Button("play##timeline_play", ImVec2(48, 0))) {
            set_playing(true);
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("<##timeline_prev", ImVec2(24, 0))) {
        select_frame(current_frame - 1);
    }
    ImGui::SameLine();
    if (ImGui::Button(">##timeline_next", ImVec2(24, 0))) {
        select_frame(current_frame + 1);
    }
    ImGui::SameLine();

    ImGui::SetNextItemWidth(80);
    int temp_frame = current_frame;
    if (ImGui::InputInt("##FrameEntry", &temp_frame, 0, 0)) {
        select_frame(temp_frame);
    }

    ImGui::SameLine();
    ImGui::Text("/ %d", total_frames);
    if (_playback_speed != 0.0F && _playback_speed != 1.0F) {
        ImGui::SameLine();
        ImGui::TextDisabled("(%.0fx)", _playback_speed);
    }
    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    ImGui::SetNextItemWidth(100.0F);
    // Displaying 0-100% is often more intuitive for users than raw float values
    float zoom_percent = _zoom_level * 100.0f;
    if (ImGui::SliderFloat("##Zoom", &zoom_percent, 0.0f, 100.0f, "Zoom: %.0f%%")) {
        _zoom_level = zoom_percent / 100.0f;
    }

    ImGui::SameLine();
    ImGui::Checkbox("Follow", &_follow_playhead);

    ImGui::End();
    ImGui::PopStyleVar(); // Pop padding for the Main Window
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::check_key_press() -> void
{
    if (ImGui::IsKeyPressed(ImGuiKey_N) && !ImGui::IsAnyItemActive()) {
        select_frame(static_cast<int>(_animation.current_frame()) + 1);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_B) && !ImGui::IsAnyItemActive()) {
        select_frame(static_cast<int>(_animation.current_frame()) - 1);
    }

    // Premiere Pro-style JKL bindings:
    //   K      -> pause
    //   L      -> play forward; each press doubles speed (1x → 2x → 4x → 8x)
    //             if currently going backward, resets to 1x forward
    //   J      -> play backward; each press doubles reverse speed (-1x → -2x → -4x → -8x)
    //             if currently going forward, resets to -1x reverse

    if (ImGui::IsKeyPressed(ImGuiKey_K) && !ImGui::IsAnyItemActive()) {
        set_playback_speed(0.0F);
    }

    if (ImGui::IsKeyPressed(ImGuiKey_L) && !ImGui::IsAnyItemActive()) {
        if (_playback_speed <= 0.0F) {
            set_playback_speed(1.0F);
        } else {
            const float next = _playback_speed * 2.0F;
            set_playback_speed(next > 8.0F ? 1.0F : next);
        }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_J) && !ImGui::IsAnyItemActive()) {
        if (_playback_speed >= 0.0F) {
            set_playback_speed(-1.0F);
        } else {
            const float next = _playback_speed * 2.0F;
            set_playback_speed(next < -8.0F ? -1.0F : next);
        }
    }

}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::current_pose() const -> std::vector<float>
{
    return _animation.current_pose();
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::current_trans() const -> std::array<float, 3>
{
    return _animation.current_trans();
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::current_frame() const -> int
{
    return static_cast<int>(_animation.current_frame());
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::select_frame(const long index) -> void
{
    _animation.set_frame(index);
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::step_frames(const long frames) -> void
{
    _animation.step_frames(frames);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::set_playing(bool playing) -> void
{
    if (playing) {
        set_playback_speed(1.0);
    } else {
        set_playback_speed(0.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::set_playback_speed(const float playback_speed) -> void
{
    _playback_speed = playback_speed;
    if (_playback_speed != 0.0F) {
        _elapsed_time = 0.0;
        _timer.start();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::tick() -> bool
{
    const auto framerate = _animation.framerate();

    // discard elapsed time
    if (not playing()) {
        return false;
    }

    // accumulate time since last animation step
    _timer.stop();
    _elapsed_time += _timer.elapsed() / 1000.0;
    _timer.start();

    const auto frame_duration = 1.0 / (framerate * std::abs(_playback_speed)); // frame duration in ms

    // new frame available
    if (_elapsed_time >= 1.0 / framerate) {
        // measure how many frames have passed since last animation step.
        const auto steps = static_cast<long>(_elapsed_time / frame_duration);
        _elapsed_time -= static_cast<double>(steps) * frame_duration;

        // apply direction from sign of playback speed
        step_frames(steps * (_playback_speed > 0.0F ? 1L : -1L));

        _dirty.store(true);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Timeline::get_dpi_scale() -> float
{
    auto imgui_io = ImGui::GetIO();
    auto dpi = imgui_io.DisplayFramebufferScale;
    std::cout << "DPI scale: " << dpi.x << ", " << dpi.y << std::endl;
    return 1.0F;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(animation)
NAMESPACE_END(volumetric_character)

// =====================================================================================================================
