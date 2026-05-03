// =====================================================================================================================

#pragma once
#ifndef VOLANIM_UTILS_MEMORY_BUFFER_H
#define VOLANIM_UTILS_MEMORY_BUFFER_H

// =====================================================================================================================

#include <streambuf>
#include <vector>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

class MemoryBuffer : public std::streambuf {
public:
    MemoryBuffer();
    ~MemoryBuffer() override;

    auto resize_buffer(size_t new_size) -> void;
    auto data() -> char*;
    auto size() -> size_t;

    // converting content to strng
    auto str() -> std::string const;

protected:
    constexpr static size_t INIT_BUFFER_SIZE { 1024 };

    int overflow(int_type ch) override;
    int underflow() override;
    int sync() override;

    // internal buffer
    std::vector<char_type> _buffer;
};


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================

#endif //VOLANIM_UTILS_MEMORY_BUFFER_H

// =====================================================================================================================
