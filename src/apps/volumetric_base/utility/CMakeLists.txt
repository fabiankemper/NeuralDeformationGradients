set(HEADERS
    constants.h
    matrix_util.hpp
    mesh_util.h
    precompute_solver.h
    utility.h
    random_util.hpp
    skinning_normalization.h
    imgui_util.h
)

set(SOURCES
    precompute_solver.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES} ${HEADERS})
