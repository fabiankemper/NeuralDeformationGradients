set(HEADERS bending_derivatives.h stretching_derivatives.h bending_energy.h stretching_energy.h)
set(SOURCES bending_energy.cpp stretching_energy.cpp)

target_sources(volumetric_skinning PUBLIC ${HEADERS} ${SOURCES})
