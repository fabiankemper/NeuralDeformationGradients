include(FetchContent)

# speed up the build
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 2f86d13775d119edbb69af52e5f566fd65c6953b
)
FetchContent_MakeAvailable(yaml-cpp)
