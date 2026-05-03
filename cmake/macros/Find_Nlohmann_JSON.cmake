# minimum cmake version: 3.11

find_package(nlohmann_json 3.11.3 QUIET)
include(FetchContent)

if (NOT nlohmann_json_FOUND)
    FetchContent_Declare(
        nlohmann_json
        DOWNLOAD_EXTRACT_TIMESTAMP ON
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif ()
