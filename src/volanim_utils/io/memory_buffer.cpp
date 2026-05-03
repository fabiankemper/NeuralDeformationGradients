// =====================================================================================================================

#include "memory_buffer.h"

// =====================================================================================================================


// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

MemoryBuffer::MemoryBuffer() : _buffer(INIT_BUFFER_SIZE)
{
    setg(_buffer.data(), _buffer.data(), _buffer.data());
    setp(_buffer.data(), _buffer.data() + _buffer.size());
}

// ---------------------------------------------------------------------------------------------------------------------

MemoryBuffer::~MemoryBuffer()
= default;

// ---------------------------------------------------------------------------------------------------------------------

auto MemoryBuffer::resize_buffer(size_t new_size) -> void
{
    // gptr = beginning of get area, eback = pointer to begin of input sequence
    std::ptrdiff_t g_offset = gptr() - eback();
    // pptr = pointer to beginning "base" of the put area, pbase = beginning pointer for output sequence
    std::ptrdiff_t p_offset = pptr() - pbase();

    _buffer.resize(new_size);

    // set read area pointers
    setg(_buffer.data(), _buffer.data() + g_offset, _buffer.data() + _buffer.size());
    // set write area pointers
    setp(_buffer.data(), _buffer.data() + _buffer.size());
    // advance write position
    pbump(p_offset);
}

// ---------------------------------------------------------------------------------------------------------------------

auto MemoryBuffer::data() -> char*
{
    return _buffer.data();
}

// ---------------------------------------------------------------------------------------------------------------------

auto MemoryBuffer::size() -> size_t
{
    return pptr() - pbase();
}

// ---------------------------------------------------------------------------------------------------------------------

auto MemoryBuffer::str() -> std::string const
{
    return std::string(pbase(), pptr());
}

// ---------------------------------------------------------------------------------------------------------------------

int MemoryBuffer::overflow(int_type ch)
{
    if (ch != EOF) {
        // std::ptrdiff_t p_offset = pptr() - pbase();
        // double memory region
        resize_buffer(_buffer.size() * 2);
        // set new character
        *pptr() = ch;
        // _buffer[p_offset] = ch;
        // advance put head
        pbump(1);
        // return new character
        return ch;
    }
    return EOF;
}

// ---------------------------------------------------------------------------------------------------------------------

int MemoryBuffer::underflow()
{
    if (gptr() < egptr()) {
        return traits_type::to_int_type(*gptr());
    }
    return EOF;
}

// ---------------------------------------------------------------------------------------------------------------------

int MemoryBuffer::sync()
{
    // no synchronization needed (no external files etc.)
    return 0;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================
