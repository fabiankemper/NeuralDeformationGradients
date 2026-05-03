set(LIBNPY_DIR "${PROJECT_SOURCE_DIR}/external/libnpy")

set(LIBNPY_SRC "${PROJECT_SOURCE_DIR}/external/libnpy/include")

set(HEADERS
    "${LIBNPY_SRC}/npy.hpp"
)

set(SOURCES
)

add_library(libnpy INTERFACE)
target_include_directories(libnpy INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external/libnpy/include>
    $<INSTALL_INTERFACE:include/libnpy> # <prefix>/include/libnpy
)
#set_target_properties(libnpy PROPERTIES
#    INTERFACE_INCLUDE_DIRECTORIES "${LIBNPY_SRC}"
#)

set(LIBNPY_FOUND 1)
