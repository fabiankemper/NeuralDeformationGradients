set(HEADERS
    mean_value_coordinates.h
    retarget_skeleton.h
)

set(SOURCES
    mean_value_coordinates.cpp
    retarget_skeleton.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES} ${HEADERS})
