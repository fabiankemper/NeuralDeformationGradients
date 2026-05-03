set(HEADERS
    defgrad_skinner.h
    vertex_skinner.h
    defgrad_predictor_onnx.h
    vertex_predictor_onnx.h
)

set(SOURCES
    defgrad_skinner.cpp
    vertex_skinner.cpp
    defgrad_predictor_onnx.cpp
    vertex_predictor_onnx.cpp
)

target_sources(volumetric_skinning PUBLIC ${SOURCES})