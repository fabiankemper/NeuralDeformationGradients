set(PMP_CHARACTER_EXT_DIR "${PROJECT_SOURCE_DIR}/external/pmp-character")
set(PMP_CHARACTER_SRC_DIR "${PMP_CHARACTER_EXT_DIR}/src/pmp_character")

## explicit version
set(HEADERS
    "${PMP_CHARACTER_SRC_DIR}/Blendshapes.h"
    "${PMP_CHARACTER_SRC_DIR}/DualQuaternion.h"
    "${PMP_CHARACTER_SRC_DIR}/InverseKinematics.h"
    "${PMP_CHARACTER_SRC_DIR}/Joint.h"
    "${PMP_CHARACTER_SRC_DIR}/MeshNode.h"
    "${PMP_CHARACTER_SRC_DIR}/PropertyNames.h"
    "${PMP_CHARACTER_SRC_DIR}/Skeleton.h"
    "${PMP_CHARACTER_SRC_DIR}/Types.h"
    "${PMP_CHARACTER_SRC_DIR}/fbxfile/fbx_export.h"
    "${PMP_CHARACTER_SRC_DIR}/fbxfile/fbx_file.h"
    "${PMP_CHARACTER_SRC_DIR}/fbxfile/fbx_warnings.h"
    "${PMP_CHARACTER_SRC_DIR}/gltf_file/gltf_file.h"
    "${PMP_CHARACTER_SRC_DIR}/gltf_file/gltf_reader.h"
    "${PMP_CHARACTER_SRC_DIR}/gltf_file/gltf_writer.h"
    "${PMP_CHARACTER_SRC_DIR}/gltf_file/tiny_gltf.h"
    "${PMP_CHARACTER_SRC_DIR}/node_types/Character.h"
    "${PMP_CHARACTER_SRC_DIR}/node_types/PointSet.h"
    "${PMP_CHARACTER_SRC_DIR}/node_types/SceneNode.h"
    "${PMP_CHARACTER_SRC_DIR}/utility/MatrixUtility.h"
    "${PMP_CHARACTER_SRC_DIR}/utility/VertexDuplicationUtilities.h"
)

set(SOURCES
    "${PMP_CHARACTER_SRC_DIR}/Blendshape.cpp"
    "${PMP_CHARACTER_SRC_DIR}/InverseKinematics.cpp"
    "${PMP_CHARACTER_SRC_DIR}/Joint.cpp"
    "${PMP_CHARACTER_SRC_DIR}/MeshNode.cpp"
    "${PMP_CHARACTER_SRC_DIR}/Skeleton.cpp"
    "${PMP_CHARACTER_SRC_DIR}/fbxfile/fbx_export.cpp"
    "${PMP_CHARACTER_SRC_DIR}/fbxfile/fbx_file.cpp"
    "${PMP_CHARACTER_SRC_DIR}/node_types/Character.cpp"
    "${PMP_CHARACTER_SRC_DIR}/node_types/PointSet.cpp"
    "${PMP_CHARACTER_SRC_DIR}/node_types/SceneNode.cpp"
)

## GLOB Version
#file(GLOB_RECURSE HEADERS
#    "${PMP_CHARACTER_EXT_DIR}/src/*.h"
#)
#file(GLOB_RECURSE SOURCES
#    "${PMP_CHARACTER_EXT_DIR}/src/*.cpp"
#)

# cleanup broken files
list(REMOVE_ITEM SOURCES "${PMP_CHARACTER_SRC_DIR}/viewer/GLTFViewer.cpp")
list(REMOVE_ITEM SOURCES "${PMP_CHARACTER_SRC_DIR}/viewer/FBXViewer.cpp")
#list(REMOVE_ITEM SOURCES "${PMP_CHARACTER_SRC_DIR}/Viewer.cpp")
#list(REMOVE_ITEM SOURCES "${PMP_CHARACTER_SRC_DIR}/gltf_file/gltf_file.cpp")

if (NOT(PMP_BUILD_VIS))
    # build STB Image yourself
    list(APPEND HEADERS
        "${PMP_CHARACTER_SRC_DIR}/stb_image/stb_image.h"
        "${PMP_CHARACTER_SRC_DIR}/stb_image/stb_image_write.h"
    )
endif ()

add_library(PMPCharacter STATIC ${SOURCES} ${HEADERS})

target_include_directories(PMPCharacter PUBLIC ${FBX_INCLUDES})
target_link_libraries(PMPCharacter PUBLIC pmp nlohmann_json::nlohmann_json ${FBX_LIBRARIES})

if (PMP_BUILD_VIS)
    # append STB Image as Library
    target_link_libraries(PMPCharacter PUBLIC stb_image stb_image_write)
endif ()

include_directories(PMPCharacter PUBLIC "${PMP_CHARACTER_EXT_DIR}/src")

set_target_properties(PMPCharacter PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PMP_CHARACTER_EXT_DIR}/src"
)

# VISUAL COMPONENTS
set(HEADERS
    "${PMP_CHARACTER_SRC_DIR}/Viewer.h"
    "${PMP_CHARACTER_SRC_DIR}/SceneGraphViewer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/CharacterRenderer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/NodeRenderer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/PointSetRenderer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/Renderer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/shaders.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/SkeletonRenderer.h"
    "${PMP_CHARACTER_SRC_DIR}/renderers/SkinnedMeshNodeRenderer.h"
)

set(SOURCES
    "${PMP_CHARACTER_SRC_DIR}/Viewer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/SceneGraphViewer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/renderers/CharacterRenderer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/renderers/NodeRenderer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/renderers/PointSetRenderer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/renderers/SkeletonRenderer.cpp"
    "${PMP_CHARACTER_SRC_DIR}/renderers/SkinnedMeshNodeRenderer.cpp"
#    "${PMP_CHARACTER_SRC_DIR}/viewer/FBXViewer.cpp"
#    "${PMP_CHARACTER_SRC_DIR}/viewer/GLTFViewer.cpp"
)
if (WITH_VISUALS)
    add_library(PMPCharacter_vis STATIC ${SOURCES} ${HEADERS})
    target_link_libraries(PMPCharacter_vis PUBLIC PMPCharacter)
    target_link_libraries(PMPCharacter_vis PUBLIC pmp_vis)
endif (WITH_VISUALS)

# PMP Character raises a lot of warnings, which we want to get rid of
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # For GCC or Clang: Add -w (disable all warnings) or other warning-suppressing flags
    target_compile_options(PMPCharacter PRIVATE -w)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # For MSVC: Add /W0 (disable all warnings)
    target_compile_options(PMPCharacter PRIVATE /W0)
endif()


set(PMPCharacter_FOUND 1)
