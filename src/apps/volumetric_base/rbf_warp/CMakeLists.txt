set(HEADERS
    rbf_warp.h
    rbf_warp_stephan.h
)

set(SOURCES

    rbf_warp_qr.cpp
    rbf_warp_stephan.cpp
)

target_sources(volumetric_base PUBLIC ${SOURCES})
