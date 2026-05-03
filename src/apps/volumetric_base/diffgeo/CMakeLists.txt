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

add_executable(test_ls_diffgeo prism_diffgeo.cpp)
target_link_libraries(test_ls_diffgeo PUBLIC volumetric_base)
target_sources(volumetric_base PUBLIC ${SOURCES} ${HEADERS})

