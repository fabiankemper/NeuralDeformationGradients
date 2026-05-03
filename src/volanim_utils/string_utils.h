//
// Created by fabian on 7/10/24.
//
// =====================================================================================================================

#pragma once
#ifndef STRING_UTILS_H
#define STRING_UTILS_H

// =====================================================================================================================

#include "volanim_utils/defs.h"

#include <filesystem>
#include <string>
#include <vector>

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================


auto string_has_ending(std::string const& fullString, std::string const& ending) -> bool;

auto string_to_lower(const std::string& string) -> std::string;

auto split(const std::string& str, const std::string& delimiter, bool remove_empty = false) -> std::vector<std::string>;

auto filename_has_extension(const std::filesystem::path& filename, const std::string& extension) -> bool;


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================

#endif //STRING_UTILS_H

// =====================================================================================================================
