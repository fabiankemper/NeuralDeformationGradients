//
// Created by gerrit on 2/10/26.
//

#ifndef VOLUMETRIC_ANIMATION_IMGUI_UTIL_H
#define VOLUMETRIC_ANIMATION_IMGUI_UTIL_H
#include "imgui.h"

namespace imgui_util {
inline void HelpMarker(const char *desc) {
  ImGui::SameLine();

  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}
} // namespace imgui_util
#endif // VOLUMETRIC_ANIMATION_IMGUI_UTIL_H
