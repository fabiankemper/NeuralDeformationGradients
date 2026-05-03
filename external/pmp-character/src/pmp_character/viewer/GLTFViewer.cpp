//
// Created by friedemannr on 04.12.23.
//


#include <filesystem>
#include "../Viewer.h"
#include "../gltf_file/gltf_file.h"

int main(int argc, char const *argv[])
{
    auto character = std::make_shared<pmp_character::Character>();
#ifdef __EMSCRIPTEN__
    std::filesystem::path meshPath("input.gltf");
#else
    if (argc < 2)
    {
        fprintf(stderr, "[USAGE] %s gltf_file\n", argv[0]);
        return -1;
    }

    std::filesystem::path meshPath(argv[1]);
#endif

    pmp_character::gltf_file gltf_file;
    gltf_file.load(character, meshPath);
    // viewer.character_ = character;
    // viewer.set_scene(character.get_center(), 0.5 * character.get_size());
    pmp_character::Viewer viewer("GLTFLoaderViewer", 800, 600, character, true);
    // viewer.load_character(character);
    return viewer.run();
}
