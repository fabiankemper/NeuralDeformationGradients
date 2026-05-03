set(HEADERS
    corotational_elastic_skinner.h
    linear_elasticity.hpp
    prism_based_corotational_elasticity.h
    corotational_utility.h
)

set(SOURCES
    corotational_elastic_skinner.cpp
    prism_based_corotational_elasticity.cpp
    corotational_utility.cpp
)

target_sources(volumetric_skinning PUBLIC ${SOURCES} ${HEADERS})
