cmake_minimum_required(VERSION 3.5.0)
project(PMPCharacter)

set(CMAKE_CXX_STANDARD 20)

# Build Options
option(BUILD_VIEWERS  "Include Viewers for Testing?" OFF)
option(BUILD_GL   "Include Renderer class for OpenGL rendering?" OFF)

set(CMAKE_CXX_FLAGS " -Wall -Wextra -Wshadow")

set(CMAKE_PROJECT_VERSION_MAJOR 1)
set(CMAKE_PROJECT_VERSION_MINOR 0)
set(CMAKE_PROJECT_VERSION_PATCH 0)
set(CMAKE_PROJECT_VERSION 1.0)

# set output paths
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

# Tell CMake where the additional Find*.cmake files are
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")

include(FetchContent)
FetchContent_Declare(
		googletest
		GIT_REPOSITORY https://github.com/google/googletest
		GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(PMP_BUILD_EXAMPLES OFF CACHE BOOL "")
set(PMP_BUILD_TEST OFF CACHE BOOL "")
set(PMP_BUILD_DOCS OFF CACHE BOOL "")

if(PMP_SCALAR_TYPE EQUAL 64)
    add_compile_definitions(PMP_SCALAR_TYPE_64)
endif()

# Run lsb_release command to get OS information
execute_process(
		COMMAND lsb_release -a
		OUTPUT_VARIABLE LSB_RELEASE_OUTPUT
		OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Print the output (for debugging purposes)
if(UNIX)
message(STATUS "lsb_release output: ${LSB_RELEASE_OUTPUT}")
endif ()
# Check if the OS is Ubuntu 20.04
if("${LSB_RELEASE_OUTPUT}" MATCHES "Ubuntu 20.04" OR  "${LSB_RELEASE_OUTPUT}" MATCHES "Ubuntu 24.04")
	message(STATUS "Operating System is Ubuntu 20.04")
	set(GLFW_BUILD_WAYLAND OFF)
else()
	message(STATUS "Operating System is not Ubuntu 20.04")
endif()


FetchContent_Declare(
		pmp
		GIT_REPOSITORY https://github.com/pmp-library/pmp-library
		GIT_TAG        e28fdc9d70c843ebd4e3cfc9f42b6a4a77e45ad4
)
FetchContent_MakeAvailable(pmp)
set_property(TARGET pmp PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET pmp_vis PROPERTY POSITION_INDEPENDENT_CODE ON)


# fbx sdk
if (NOT EMSCRIPTEN)
	find_package(FBX)

	if (NOT FBX_FOUND)
	    message("FBX SDK not found")
	else()
	    include_directories(${FBX_INCLUDE_DIRS})
	endif()
endif()

# Emscripten options copied from pmp CMake
if(EMSCRIPTEN)
  add_compile_options(--no-heap-copy)
  add_compile_options(-fexceptions)
  add_link_options(-fexceptions)
  add_link_options(
    "SHELL:-sWASM=1 -sUSE_WEBGL2=1 -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sALLOW_TABLE_GROWTH=1 -sSTACK_SIZE=5MB"
  )
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

add_subdirectory(src/pmp_character)
add_subdirectory(tests)

