
# check for open MP support
if(NOT USE_OPEN_MP STREQUAL "ON" OR APPLE OR NOT OpenMP_FOUND)
    message(STATUS "Disabled OpenMP support.")
    add_compile_definitions(NO_USE_OPENMP)
else()
    message(STATUS "Enabled OpenMP support.")
endif ()
