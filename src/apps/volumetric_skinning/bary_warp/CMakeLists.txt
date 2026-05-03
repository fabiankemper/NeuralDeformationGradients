set(HEADERS
    bary_warp.h
    barycentric_warp.h
)

set(SOURCES
    barycentric_warp.cpp
    bary_warp.cpp
)

target_sources(volumetric_skinning PUBLIC ${SOURCES} ${HEADERS})

add_executable(barywarp_preprocess barywarp_preprocess.cpp)
target_link_libraries(barywarp_preprocess PUBLIC volumetric_skinning volumetric_base)