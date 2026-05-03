# set stark options

set(STARK_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Python Bindings" FORCE)

if (APPLE)
    set(STARK_ENABLE_AVX OFF CACHE BOOL "Flag to enable AVX for Stark and its dependencies" FORCE)
endif ()