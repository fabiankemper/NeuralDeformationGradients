set(HEADERS
    skinning_base.h
    collision_detection.h
    prism_adapter.h
    energy_base.h
    io.hpp
    hessian_constructor.h
    basic_skinning.h
    volumetric_boundary_fixer.h
)

set(SOURCES
    skinning_base.cpp
    collision_detection.cpp
    prism_adapter.cpp
    energy_base.cpp
    basic_skinning.cpp
    volumetric_boundary_fixer.cpp
)

target_sources(volumetric_skinning PUBLIC ${SOURCES} ${HEADERS})