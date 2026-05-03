# Intel MKL. This is ugly
if (MSVC)
    set(MKL_THREADING "sequential")
else ()
    set(MKL_THREADING "gnu_thread")
endif ()


if (NOT MSVC)
    set(MKL_DIR "${MKL_ROOT}/lib/cmake/mkl")
    find_package(MKL QUIET)
    if(NOT MKL_FOUND)
        message("Could not find MKL, instead building with slower Eigen Solver")
    endif ()
endif ()

message(STATUS "${MKL_IMPORTED_TARGETS}")
if (MKL_FOUND)
    message(STATUS "Found Intel MKL.")
endif ()
