# ufbx v0.23.1 (https://github.com/ufbx/ufbx), single source file FBX loader
set(UFBX_DIR "${PROJECT_SOURCE_DIR}/external/ufbx")

add_library(ufbx STATIC)
target_sources(ufbx PRIVATE "${UFBX_DIR}/ufbx.c" "${UFBX_DIR}/ufbx.h")
target_include_directories(ufbx PUBLIC "${UFBX_DIR}")
