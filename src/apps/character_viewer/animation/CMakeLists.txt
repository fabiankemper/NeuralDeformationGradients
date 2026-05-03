set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/timeline.h
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/timeline.cpp
)

target_sources(character_viewer PUBLIC "${HEADERS}" "${SOURCES}")
