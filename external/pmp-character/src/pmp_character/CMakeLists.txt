set(LIBRARIES pmp)

set(SOURCES
    node_types/Character.cpp
    InverseKinematics.cpp
    Joint.cpp
    MeshNode.cpp
    Skeleton.cpp
    Blendshape.cpp
    gltf_file/gltf_file.cpp
    node_types/SceneNode.cpp
    node_types/PointSet.cpp
)

set(HEADERS
        node_types/Character.h
    InverseKinematics.h
    Joint.h
    MeshNode.h
    Skeleton.h
    DualQuaternion.h
    Blendshapes.h
    gltf_file/gltf_file.h
    gltf_file/gltf_writer.h
    gltf_file/gltf_reader.h
    stb_image/stb_image.h
    stb_image/stb_image_write.h
    gltf_file/tiny_gltf.h
    json/json.hpp
    utility/MatrixUtility.h
    node_types/SceneNode.h
    node_types/PointSet.h

)

if (FBX_FOUND)
    set(SOURCES ${SOURCES} fbxfile/fbx_file.cpp)
    set(HEADERS ${HEADERS} fbxfile/fbx_file.h)
    set(LIBRARIES ${LIBRARIES} ${FBX_LIBRARIES})
endif()

if(BUILD_GL)
    set(SOURCES ${SOURCES}
        renderers/CharacterRenderer.cpp
        renderers/SkinnedMeshNodeRenderer.cpp
        renderers/SkeletonRenderer.cpp
        renderers/PointSetRenderer.cpp
        Viewer.cpp
        utility/VertexDuplicationUtilities.h
    )

    set(HEADERS ${HEADERS}
        renderers/CharacterRenderer.h
        renderers/SkinnedMeshNodeRenderer.h
        renderers/PointSetRenderer.h
        renderers/SkeletonRenderer.h
        Viewer.h
        renderers/shaders.h
        renderers/NodeRenderer.cpp
        renderers/NodeRenderer.h
        SceneGraphViewer.cpp
        SceneGraphViewer.h
        renderers/Renderer.h
    )

    set(LIBRARIES ${LIBRARIES} pmp_vis imgui)
endif()

add_subdirectory(node_types)
message("Headers: ${HEADERS}")
add_library(pmp_character ${HEADERS} ${SOURCES})

target_link_libraries(pmp_character PUBLIC ${LIBRARIES})
if(BUILD_VIEWERS AND BUILD_GL)
    if (FBX_FOUND)
        add_executable(FBXViewer viewer/FBXViewer.cpp)
        target_link_libraries(FBXViewer PUBLIC pmp_character)
    endif()

    add_executable(GLTFViewer viewer/GLTFViewer.cpp)
    target_link_libraries(GLTFViewer PUBLIC pmp_character)

    if (EMSCRIPTEN)
        set_target_properties(GLTFViewer
           PROPERTIES
             LINK_FLAGS
             "--shell-file ${CMAKE_SOURCE_DIR}/data/shell.html\
              --preload-file ${CMAKE_SOURCE_DIR}/tests/data/boerner.gltf@input.gltf\
              --preload-file ${CMAKE_SOURCE_DIR}/tests/data/boerner.bin@boerner.bin\
              --preload-file ${CMAKE_SOURCE_DIR}/tests/data/skin_basecolor.png@skin_basecolor.png\
              --preload-file ${CMAKE_SOURCE_DIR}/data/default_vitras_template.dof@default_vitras_template.dof"
         )
    endif()

endif()

# Install targets
target_include_directories(pmp_character
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/>
)

install(TARGETS pmp_character
        EXPORT pmp_characterTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

install(FILES ${HEADERS} DESTINATION include/pmp_character/)
