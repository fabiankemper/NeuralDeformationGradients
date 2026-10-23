
add_library(volumetric_skinning)


if (PARALLEL_SKINNING)
    target_compile_definitions(volumetric_skinning PUBLIC PARA_SKINNING)
endif ()

# add all headers as include
target_include_directories(volumetric_skinning PUBLIC "${PROJECT_SOURCE_DIR}/src/apps")

# add intel mkl
if (MKL_FOUND)
    target_link_libraries(volumetric_skinning PUBLIC MKL::MKL)
    target_compile_definitions(volumetric_skinning PUBLIC USE_MKL)
endif ()

target_link_libraries(volumetric_skinning PUBLIC volanim_utils)
target_link_libraries(volumetric_skinning PUBLIC volumetric_base)
target_link_libraries(volumetric_skinning PUBLIC volumetric_character)
target_link_libraries(volumetric_skinning PUBLIC fast_rotation_fitting)

if (PMP_BUILD_VIS)
    target_compile_definitions(volumetric_skinning PUBLIC WITH_IMGUI)
    target_link_libraries(volumetric_skinning PUBLIC imgui)
endif ()


if (TORCH_FOUND)
    target_link_libraries(volumetric_skinning PUBLIC "${TORCH_LIBRARIES}")
endif ()
target_link_onnxruntime(volumetric_skinning)

add_subdirectory(base)
add_subdirectory(corotational_linear_elasticity)
add_subdirectory(neural_skinning)
add_subdirectory(projective_skinning)
add_subdirectory(bary_warp)
add_subdirectory(stable_neo_hookean_elasticity)
add_subdirectory(thin_shell)
