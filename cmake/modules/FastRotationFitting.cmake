set(FAST_ROTATION_FITTING_DIR "${PROJECT_SOURCE_DIR}/external/fast_rotation_fitting")
set(FAST_ROTATION_FITTING_INC_DIR "${PROJECT_SOURCE_DIR}/external")

set(HEADERS
    ${FAST_ROTATION_FITTING_DIR}/cayley_rotation_fitting_util.hpp
    ${FAST_ROTATION_FITTING_DIR}/fit_rotation_avx.h
)

set(SOURCES
    ${FAST_ROTATION_FITTING_DIR}/fit_rotation_avx.cpp
)

add_library(fast_rotation_fitting STATIC)
target_sources(fast_rotation_fitting PUBLIC ${SOURCES} ${HEADERS})
target_include_directories(fast_rotation_fitting PUBLIC "${FAST_ROTATION_FITTING_INC_DIR}")
