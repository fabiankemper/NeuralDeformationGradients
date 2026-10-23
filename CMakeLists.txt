cmake_minimum_required(VERSION 3.22)

project(volumetric_animation)

# cmake policies
include(cmake/CmakePolicies.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set output paths
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/")

# set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# custom options
set(WITH_VISUALS            ON  CACHE BOOL "Enable all visual targets in the project")

# set compiler options
include(cmake/CompilerOptions.cmake)

# pmp-library
set(PMP_BUILD_EXAMPLES     OFF CACHE BOOL "")
set(PMP_BUILD_TESTS        OFF CACHE BOOL "")
set(PMP_BUILD_DOCS         OFF CACHE BOOL "")
set(PMP_INSTALL            OFF CACHE BOOL "")
set(PMP_STRICT_COMPILATION ON  CACHE BOOL "")
set(BUILD_SHARED_LIBS      OFF CACHE BOOL "")
set(USE_OPEN_MP            ON  CACHE BOOL "")

if (NOT DEFINED PMP_BUILD_VIS)
    set(PMP_BUILD_VIS ${WITH_VISUALS})
endif ()

# disable AVX on macOS
if (APPLE)
    set(STARK_ENABLE_AVX OFF)
endif ()

add_subdirectory(${PROJECT_SOURCE_DIR}/external/pmp-library)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/src)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/eigen-3.4.0)
#include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/glew-2.2.0/include)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/glad-2.0.8/include)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/glfw-3.4/include)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/imgui-1.91.8)

# do not treat warnings as errors (operator new bug)
target_compile_options(pmp PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-error>
        $<$<CXX_COMPILER_ID:MSVC>:/WX-> # disables /WX on MSVC
)
if (WITH_VISUALS)
    target_compile_options(pmp_vis PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-error>
            $<$<CXX_COMPILER_ID:MSVC>:/WX->
    )
endif ()


# external libraries with source
include(cmake/modules/FastRotationFitting.cmake)

# cmake modules
include(FetchContent)

# ufbx (fbx loading)
include(cmake/modules/UFBX.cmake)

# nlohmann json
include(cmake/macros/Find_Nlohmann_JSON.cmake)

# argparse
include(cmake/modules/Argparse.cmake)

# libnpy (numpy loading)
include(cmake/modules/FindLibNPY.cmake)

# libzip for libz files
include(cmake/modules/FindLibZippp.cmake)

# Intel MKL for faster solver
include(cmake/modules/FindIntelMKL.cmake)

# ONNX
include(cmake/modules/Find_ONNX.cmake)

# debugging purposes
add_compile_definitions(PRINT_INFOS)

# find openmp by homebrew
if(APPLE)
    set(HOMEBREW_PREFIX "/opt/homebrew" CACHE PATH "Path to Homebrew installation")
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" "${HOMEBREW_PREFIX}/opt/libomp")
endif()

# openMP
find_package(OpenMP)
include(cmake/functions/ConfigureOpenMP.cmake)


# pmp character
# note: PMP Character must be loaded after PMP Library
find_package(PMPCharacter)

if (NOT PMPCharacter_FOUND)
    message(FATAL_ERROR "PMPCharacter not found")
endif ()

#stupid MS bugs
if (MSVC)
    add_compile_options(/FS)
endif()

include_directories(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(src)