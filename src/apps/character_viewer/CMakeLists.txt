set(SOURCES
    main.cpp
    character_viewer.cpp
    constants.cpp
)

set(HEADERS
    character_viewer.h
    constants.h
)

# main executable
add_executable(character_viewer ${SOURCES})

# Check the compiler and set warning flags accordingly
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(character_viewer PRIVATE -Wall -Wextra -Wpedantic)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(character_viewer PRIVATE /W4)
endif()

# libraries
target_link_libraries(character_viewer PUBLIC PMPCharacter_vis)
target_link_libraries(character_viewer PUBLIC argparse libnpy)
target_link_libraries(character_viewer PUBLIC volumetric_character_vis)

# include dir
target_include_directories(character_viewer PUBLIC
    "${PROJECT_SOURCE_DIR}/src/apps"
)

add_subdirectory(animation)
