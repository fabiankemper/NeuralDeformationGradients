include(FetchContent)

# require cuda toolkit for non-apple devices
if (NOT APPLE)
    find_package(CUDAToolkit REQUIRED)
endif ()

# --- try finding onnx runtime on the system ---
message(STATUS "Checking for system-provided ONNX runtime ...")

# TIER 1: try to find onnxruntime installed via find_package
find_package(onnxruntime QUIET)

if(onnxruntime_FOUND AND TARGET onnxruntime)
    message(STATUS "  -> Found via CMake Config (${onnxruntime_DIR})")
    # Extract properties from the provided target to keep our setup uniform
    get_target_property(ONNXRUNTIME_SYS_INCLUDE_DIR onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(ONNXRUNTIME_SYS_LIBRARY onnxruntime IMPORTED_LOCATION)

    # Some configs use LOCATION instead of IMPORTED_LOCATION
    if(NOT ONNXRUNTIME_SYS_LIBRARY)
        get_target_property(ONNXRUNTIME_SYS_LIBRARY onnxruntime LOCATION)
    endif()
endif()

# TIER 2: look for main header file (e.g. in /opt/homebrew for macOS Apple Silicon)
if(NOT ONNXRUNTIME_SYS_INCLUDE_DIR OR NOT ONNXRUNTIME_SYS_LIBRARY)
    # Look for the main header file
    find_path(ONNXRUNTIME_SYS_INCLUDE_DIR onnxruntime_cxx_api.h
        PATH_SUFFIXES onnxruntime core/session
        PATHS
        /usr/include
        /usr/local/include
        /opt/homebrew/include
        /opt/onnxruntime-cuda/include
        /opt/onnxruntime-opt-cuda/include
        /opt/onnxruntime/include
    )

    # Look for the shared library
    find_library(ONNXRUNTIME_SYS_LIBRARY NAMES onnxruntime
        PATHS
        /usr/lib
        /usr/local/lib
        /opt/homebrew/lib
        /opt/onnxruntime-cuda/lib
        /opt/onnxruntime-opt-cuda/lib
        /opt/onnxruntime/lib
    )
endif()

# --- setup onnx runtime target ---

add_library(ONNXRuntime SHARED IMPORTED GLOBAL)
if(ONNXRUNTIME_SYS_INCLUDE_DIR AND ONNXRUNTIME_SYS_LIBRARY)
    set_target_properties(ONNXRuntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_SYS_INCLUDE_DIR}"
        IMPORTED_LOCATION "${ONNXRUNTIME_SYS_LIBRARY}"
    )

    # Extract the directory of the found library to use in RPATH later
    get_filename_component(ORT_RPATH_DIR "${ONNXRUNTIME_SYS_LIBRARY}" DIRECTORY)
    set(ORT_SOURCE_TYPE "System Installed")

else()
    # Tier 3: Fallback to FetchContent
    set(ORT_VERSION "1.20.0")
    if(WIN32)
        set(ORT_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-win-x64-gpu-${ORT_VERSION}.zip")
    elseif(APPLE)
        set(ORT_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-osx-universal2-${ORT_VERSION}.tgz")
    elseif(UNIX)
        set(ORT_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-gpu-${ORT_VERSION}.tgz")
    else()
        message(FATAL_ERROR "This automatic setup only supports Windows, macOS, and Linux (x64).")
    endif()

    FetchContent_Declare(
        onnxruntime
        URL ${ORT_URL}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(onnxruntime)

    set_target_properties(ONNXRuntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_SOURCE_DIR}/include"
    )

    if (WIN32)
        set_target_properties(ONNXRuntime PROPERTIES
            IMPORTED_IMPLIB   "${onnxruntime_SOURCE_DIR}/lib/onnxruntime.lib"
            IMPORTED_LOCATION "${onnxruntime_SOURCE_DIR}/lib/onnxruntime.dll"
        )
        set(ONNXRUNTIME_SYS_LIBRARY "${onnxruntime_SOURCE_DIR}/lib/onnxruntime.dll")
    elseif(APPLE)
        set_target_properties(ONNXRuntime PROPERTIES
            IMPORTED_LOCATION "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.dylib"
        )
        set(ONNXRUNTIME_SYS_LIBRARY "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.dylib")
    else()
        set_target_properties(ONNXRuntime PROPERTIES
            IMPORTED_LOCATION "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so"
        )
        set(ONNXRUNTIME_SYS_LIBRARY "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so")
    endif()

    set(ORT_RPATH_DIR "${onnxruntime_SOURCE_DIR}/lib")
    set(ONNXRUNTIME_SYS_INCLUDE_DIR "${onnxruntime_SOURCE_DIR}/include")
    set(ORT_SOURCE_TYPE "Fetched from Web (v${ORT_VERSION})")
endif()


# --- helper function to link ONNX runtime and handle runtime setup

function(target_link_onnxruntime TARGET_NAME)
    target_link_libraries(${TARGET_NAME} PUBLIC ONNXRuntime)

    if(WIN32)
        if(_target_type STREQUAL "EXECUTABLE" OR
                _target_type STREQUAL "SHARED_LIBRARY" OR
                _target_type STREQUAL "MODULE_LIBRARY")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${TARGET_NAME}>
                    $<TARGET_FILE_DIR:${TARGET_NAME}>
                    COMMAND_EXPAND_LISTS
            )
        endif()
    elseif(APPLE)
        set_target_properties(${TARGET_NAME} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "${ORT_RPATH_DIR}"
        )
    else()
        set_target_properties(${TARGET_NAME} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "${ORT_RPATH_DIR};${CUDAToolkit_LIBRARY_DIR}"
        )
    endif()
endfunction()

# ---- print status ----

message(STATUS "=======================================================")
message(STATUS "              ONNX Runtime Configuration               ")
message(STATUS "=======================================================")
message(STATUS "  Source:      ${ORT_SOURCE_TYPE}")
message(STATUS "  Includes:    ${ONNXRUNTIME_SYS_INCLUDE_DIR}")
message(STATUS "  Library:     ${ONNXRUNTIME_SYS_LIBRARY}")
if(APPLE)
    message(STATUS "  Hardware:    CPU / CoreML (macOS)")
else()
    message(STATUS "  Hardware:    CUDA GPU Expected")
endif()
message(STATUS "=======================================================")