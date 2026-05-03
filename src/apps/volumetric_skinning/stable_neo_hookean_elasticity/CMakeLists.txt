set(HEADERS neo_hookean_derivatives.h
            StableNeoHookeanSimulation.h
            StableNeoHookeanSkinner.h
            NeoHookeanEnergy.h
            NeoHookeanEnergyQuadrature.h
            StableNeoHookeanTets.h
            invariants_and_derivatives.h)
set(SOURCES StableNeoHookeanSimulation.cpp
            StableNeoHookeanSkinner.cpp
            NeoHookeanEnergy.cpp
            NeoHookeanEnergyQuadrature.cpp
            StableNeoHookeanTets.cpp)


target_sources(volumetric_skinning PUBLIC ${HEADERS} ${SOURCES})