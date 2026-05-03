#set(FBX_EXT_DIR "${PROJECT_SOURCE_DIR}/external/fbxsdk")
set(FBX_EXT_DIR "/usr")

if(APPLE)
    find_file(FBX_LIBRARY libfbxsdk.a "${FBX_EXT_DIR}/lib/macos/clang/")
elseif(UNIX)
    find_file(FBX_LIBRARY libfbxsdk.a "${FBX_EXT_DIR}/lib/linux/")
elseif(WIN32)
    find_file(FBX_LIBRARY libfbxsdk.lib "${FBX_EXT_DIR}/lib/windows/")
    # preprocessor option for FBXSDK on windows
    # reference: https://help.autodesk.com/view/FBX/2018/ENU/?guid=FBX_Developer_Help_getting_started_installing_and_configuring_configuring_the_fbx_sdk_for_wind_html
    add_compile_definitions(FBXSDK_SHARED)
    configure_file("${FBX_EXT_DIR}/lib/windows/libfbxsdk.dll" ${CMAKE_BINARY_DIR}/libfbxsdk.dll COPYONLY)
else()
    message(FATAL_ERROR "Unknown OS for FBX loader.")
endif()


find_file(FBX_LIBRARY libfbxsdk.so "${FBX_EXT_DIR}/lib/")
find_file(FBX_INCLUDE fbxsdk.h "${FBX_EXT_DIR}/include/")

if(NOT FBX_LIBRARY OR NOT FBX_INCLUDE)
    set(FBX_FOUND 0)
    unset(FBX_LIB)
    unset(FBX_INCLUDE)
    message(WARNING "FBXSDK NOT found in ${PROJECT_SOURCE_DIR}/external/fbxsdk/. Please make sure to copy libfbxsdk.a in external/fbxsdk/lib/ and the header files to external/fbxsdk/include/ if you want to write fbx files!")
else()
    if(NOT WIN32)
        find_package(LibXml2 REQUIRED)
        find_package(ZLIB REQUIRED)
    else()
        find_file(ZLIB_LIBRARIES libxml2.lib "${FBX_EXT_DIR}/lib/windows/")
        find_file(ZLIB_LIBRARIES zlib1.lib "${FBX_EXT_DIR}/lib/windows/")
    endif()

    set(FBX_FOUND 1)
    set(FBX_INCLUDE_DIRS "${FBX_EXT_DIR}/include/")
    set(FBX_LIBRARIES ${FBX_LIBRARY} ${LIBXML2_LIBRARIES} ${ZLIB_LIBRARIES})

    if(APPLE)
        find_package(Iconv MODULE REQUIRED)
        set(CARBON_LIBRARIES /System/Library/Frameworks/Carbon.framework)
        set(FBX_LIBRARIES ${FBX_LIBRARIES} ${Iconv_LIBRARIES} ${CARBON_LIBRARIES})
    endif()

    add_definitions(-DHAVE_FBX)
endif()


message(STATUS "FBX Include ${FBX_INCLUDE}")
message(STATUS "FBX LIB ${FBX_LIBRARY}")
