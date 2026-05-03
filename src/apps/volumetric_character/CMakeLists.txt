add_library(volumetric_character)
# VISUAL TARGET
add_library(volumetric_character_vis)

set(HEADERS
    character_utils.h
    volumetric_character.h
)

set(SOURCES
    volumetric_character.cpp
)

target_sources(volumetric_character PUBLIC ${SOURCES} ${HEADERS})
target_link_libraries(volumetric_character PUBLIC PMPCharacter)
target_link_libraries(volumetric_character PUBLIC libnpy)
target_link_libraries(volumetric_character PUBLIC volumetric_skinning)
target_link_libraries(volumetric_character_vis PUBLIC volumetric_character)

# include directory for all apps
target_include_directories(volumetric_character PUBLIC "${PROJECT_SOURCE_DIR}/src/apps")
target_include_directories(volumetric_character_vis PUBLIC "${PROJECT_SOURCE_DIR}/src/apps")

# include directory for parent project
#set_target_properties(volumetric_character PROPERTIES
#    INTERFACE_INCLUDE_DIRECTORIES "${PMP_CHARACTER_EXT_DIR}/src/apps"
#)

# FBX SDK
target_include_directories(volumetric_character PUBLIC ${FBX_INCLUDES})
target_link_libraries(volumetric_character PUBLIC ${FBX_LIBRARIES})
# note, we do not add the "volumetric_character" directory as prefix
#target_include_directories(volumetric_character PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


add_subdirectory(renderer)
add_subdirectory(skinning)
