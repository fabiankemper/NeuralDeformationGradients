set(HEADERS
    eigen_sparse.hpp
    io_npz.h
    io_selection.h
    memory_buffer.h
)

set(SOURCES
    eigen_sparse.cpp
    io_npz.cpp
    io_selection.cpp
    memory_buffer.cpp
)

target_sources(volanim_utils PUBLIC ${SOURCES} ${HEADERS})
