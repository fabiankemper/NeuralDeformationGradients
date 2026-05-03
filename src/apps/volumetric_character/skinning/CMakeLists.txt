set(HEADERS
    dqs.h
    lbs.h
    tet_volumetric.h
)

set(SOURCES
    dqs.cpp
    lbs.cpp
    tet_volumetric.cpp
)

target_sources(volumetric_character PUBLIC ${SOURCES} ${HEADERS})
