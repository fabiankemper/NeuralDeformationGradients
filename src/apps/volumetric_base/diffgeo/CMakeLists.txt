set(HEADERS
    tetrahedral.h
    prism_diffgeo.h
    diffgeo.h
)

set(SOURCES
    tetrahedral.cpp
    prism_diffgeo.cpp
    diffgeo.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES} ${HEADERS})

