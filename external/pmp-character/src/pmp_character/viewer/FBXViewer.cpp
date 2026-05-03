//
// Created by friedemannr on 04.12.23.
//

#include <filesystem>
#include "../Viewer.h"
#include "../fbxfile/fbx_file.h"

int main(int argc, char const *argv[])
{
    if (argc < 2)
    {
        std::cerr << "[USAGE]: " << argv[0] << " /path/to/character.fbx" << std::endl;
        return -1;
    }
    pmp_character::FBX_file fbx_file;
    pmp_character::Character character;
    std::filesystem::path meshPath(argv[1]);
    fbx_file.read(&character, meshPath);
    // viewer.character_ = character;
    // viewer.set_scene(character.get_center(), 0.5 * character.get_size());
    pmp_character::Viewer viewer("FBXLoaderViewer", 1920, 1080, &character);
    viewer.run();
}
