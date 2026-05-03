SET(HEADERS
    configuration.h
    filenames.h
    io.h
    utils.h
    volumetric_state.h
)

set(SOURCES
    configuration.cpp
    filenames.cpp
    io.cpp
    utils.cpp
    volumetric_state.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES})

add_subdirectory(retarget_skeleton)
add_subdirectory(skinning)
