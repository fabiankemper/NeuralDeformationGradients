//
// Created by fabian on 7/11/24.
//
// =====================================================================================================================

#include "string_utils.h"

#include <algorithm>

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto string_has_ending (std::string const& fullString, std::string const& ending) -> bool {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto string_to_lower(const std::string& string) -> std::string
{
    std::string result = string;
    std::ranges::transform(result, result.begin(), [](const unsigned char c) { return static_cast<unsigned char>(std::tolower(c)); });
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto ends_with(const std::string& str, const std::string& suffix) -> bool
{
    return str.size() >= suffix.size() && str.substr(str.size() - suffix.size()) == suffix;
}

// ---------------------------------------------------------------------------------------------------------------------

auto split(const std::string& str, const std::string& delimiter, bool remove_empty) -> std::vector<std::string>
{
    std::vector<std::string> tokens;
    for (size_t start = 0, end; start < str.length(); start = end + delimiter.length()) {
        size_t pos = str.find(delimiter, start);
        end = pos != std::string::npos ? pos : str.length();

        std::string token = str.substr(start, end - start);
        if (!remove_empty || !token.empty()) {
            tokens.push_back(token);
        }
    }

    if (!remove_empty && (str.empty() || ends_with(str, delimiter))) {
        tokens.push_back(str);
    }

    return tokens;
}

// ---------------------------------------------------------------------------------------------------------------------

auto filename_has_extension(const std::filesystem::path& filename, const std::string& extension) -> bool
{
    const std::string file_ext = string_to_lower(filename.extension().string());
    return file_ext == "." + extension;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================