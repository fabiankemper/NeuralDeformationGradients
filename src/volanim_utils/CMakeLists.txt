set(HEADERS
    defs.h
    compiler_support.h
    lru_cache.hpp
    matrix_utils.h
    npy_utils.h
    random_utils.h
    string_utils.h
)

set(SOURCES
    matrix_utils.cpp
    npy_utils.cpp
    random_utils.cpp
    string_utils.cpp
)

add_library(volanim_utils)

target_sources(volanim_utils PUBLIC ${SOURCES})

target_link_libraries(volanim_utils PUBLIC pmp)
target_link_libraries(volanim_utils PUBLIC libzippp libnpy)
target_include_directories(volanim_utils PRIVATE "${PROJECT_SOURCE_DIR}/src/apps")

add_subdirectory(io)
