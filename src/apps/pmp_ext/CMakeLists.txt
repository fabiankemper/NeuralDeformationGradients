# PMP Extensions (make internal structures public)
# should be renamed to pmp utils
add_library(pmp_ext STATIC)

set(HEADERS
    bsp_tree.h
    defs.h
    mesh_properties.h
    normals.h
    pmp_io.h
    pmp_matrix_ext.h
    pmp_utility.h
    priority_queue.h
    triangle_kd_tree.h
)

set(SOURCES
    bsp_tree.cpp
    normals.cpp
    pmp_io.cpp
    pmp_matrix_ext.cpp
    pmp_utility.cpp
    triangle_kd_tree.cpp
)

target_sources(pmp_ext PUBLIC ${SOURCES} ${HEADERS})
target_link_libraries(pmp_ext PUBLIC PMPCharacter)

# include directory for all apps
target_include_directories(pmp_ext PUBLIC "${PROJECT_SOURCE_DIR}/src/apps")

