
include(FetchContent)

# source: https://github.com/madler/zlib/issues/759#issuecomment-1384244323
include(cmake/functions/FetchContent_MakeAvailableExcludeFromAll.cmake)


# zlib
# find_package(ZLIB QUIET)
if (NOT ${ZLIB_FOUND})
    FetchContent_Declare(zlib
        URL     https://www.zlib.net/zlib-1.3.1.tar.gz
                https://www.zlib.net/fossils/zlib-1.3.1.tar.gz
                https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz
        URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
    )
    # FetchContent_MakeAvailable(zlib)
    FetchContent_MakeAvailableExcludeFromAll(zlib)

    if (VOLANIM_STATIC)
        add_library(ZLIB::ZLIB ALIAS zlibstatic)
        target_include_directories(zlibstatic INTERFACE ${zlib_BINARY_DIR} ${zlib_SOURCE_DIR})
    else()
        add_library(ZLIB::ZLIB ALIAS zlib)
        target_include_directories(zlib INTERFACE ${zlib_BINARY_DIR} ${zlib_SOURCE_DIR})
    endif()

    # 1. Set the variables that are usually set by FindZLIB.cmake.
    # 2. Add the module that stubs out `find_package(ZLIB ...)` calls.
    set(ZLIB_FOUND ON)
    set(ZLIB_LIBRARY ZLIB::ZLIB)
    set(ZLIB_LIBRARIES ZLIB::ZLIB)
    set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
    set(ZLIB_INCLUDE_DIRS ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
    set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake")
endif ()


# libzip
find_package(LIBZIP QUIET)
if (NOT ${LIBZIP_FOUND})
    FetchContent_Declare(libzip
        GIT_REPOSITORY      "https://github.com/nih-at/libzip.git"
        GIT_TAG             "v1.10.1"
    )
    FetchContent_MakeAvailable(libzip)
endif ()


# libzippp
FetchContent_Declare(libzippp
    GIT_REPOSITORY    "https://github.com/ctabin/libzippp.git" # can also be a local filesystem path!
    #GIT_TAG           "master"
    GIT_TAG           "libzippp-v7.1-1.10.1"
)
FetchContent_MakeAvailable(libzippp) # defines INTERFACE target libzippp
