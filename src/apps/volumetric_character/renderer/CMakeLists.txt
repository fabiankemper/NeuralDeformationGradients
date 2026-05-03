set(HEADERS
    skeleton_renderer.h
    skinnable_mesh_renderer.h
    volumetric_character_renderer.h
    warped_mesh_renderer.h
)

set(SOURCES
    skeleton_renderer.cpp
    skinnable_mesh_renderer.cpp
    volumetric_character_renderer.cpp
    warped_mesh_renderer.cpp
)

target_sources(volumetric_character_vis PUBLIC ${SOURCES} ${HEADERS})

add_subdirectory(shader)
