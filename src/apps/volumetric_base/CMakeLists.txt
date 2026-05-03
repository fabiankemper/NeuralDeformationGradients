add_library(volumetric_base
        state/skinning/skinning.cpp
        state/skinning/skinning.h)

set(HEADERS
    aliases.h
    animation.h
    layered_mesh.h
    skinnable_mesh.h
    state/volumetric_state.h
    state/utils.h
    volumetric_types.h
)

set(SOURCES
    animation.cpp
    layered_mesh.cpp
    skinnable_mesh.cpp
    state/utils.cpp
    state/volumetric_state.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES})
target_link_libraries(volumetric_base PUBLIC pmp_ext)
target_link_libraries(volumetric_base PUBLIC volanim_utils)
target_link_libraries(volumetric_base PUBLIC nlohmann_json::nlohmann_json)

# include directory for all apps
target_include_directories(volumetric_base PRIVATE "${PROJECT_SOURCE_DIR}/src/apps")

add_subdirectory(io)
add_subdirectory(diffgeo)
add_subdirectory(rbf_warp)
add_subdirectory(state)
add_subdirectory(utility)

# fix for windows
if (MSVC)
    target_compile_options(volumetric_base PUBLIC "-openmp:experimental")
endif (MSVC)
