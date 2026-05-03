
# todo: find platform independent option
#set(COMMON_CXX_FLAGS "-pedantic -Wall -Wextra -Wshadow")
#set(CMAKE_CXX_FLAGS ${COMMON_CXX_FLAGS})
message("Type is ${CMAKE_BUILD_TYPE}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    add_compile_definitions(-DNOMINMAX)
    add_compile_definitions(_USE_MATH_DEFINES)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    # set(BUILD_SHARED_LIBS ON)
endif()

include(CheckCXXCompilerFlag)

macro(check_and_add_flag flag variable)
    check_cxx_compiler_flag(${flag} ${variable})
    message(STATUS "Checking ${flag} support.")
    if(${${variable}})
        add_compile_options(${flag})
        message(STATUS "Added ${flag}.")
    else()
        message("${flag} not supported.")
    endif ()
endmacro()

macro(target_check_and_add_flag target flag variable)
    check_cxx_compiler_flag(${flag} ${variable})
    if(${${variable}})
        message( "Adding ${flag} to ${target}" )
        target_compile_options(${target} PUBLIC ${flag})
    endif()
endmacro()


# check for avx2
include("./cmake/macros/Find_AVX.cmake")

# add avx flags if supported by this machine
CHECK_FOR_AVX()
if(AVX_FLAGS)
    add_compile_options(${AVX_FLAGS})
endif()
if(HAVE_AVX2_EXTENSIONS)

    add_compile_definitions("HAVE_AVX2")
    add_compile_options(${AVX2_FLAGS})
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-stringop-overread)
    add_compile_options(-Wno-array-bounds)
endif ()
# Add optimization flags in Release mode
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        add_compile_options(/O2)
    else()
        check_and_add_flag("-O3" HAVE_CXX_O3)
        check_and_add_flag("-mtune=native" HAVE_CXX_NATIVE)
    endif()
endif()

