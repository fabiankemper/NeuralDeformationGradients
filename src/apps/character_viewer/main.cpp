// =====================================================================================================================

#include <argparse/argparse.hpp>

#include "volumetric_base/state/configuration.h"

#include "character_viewer/character_viewer.h"
#include "character_viewer/constants.h"
#ifdef USE_MKL
#include <mkl.h>
#endif
// =====================================================================================================================

auto parse_cmd_line(const argparse::ArgumentParser& program) -> bool
{
    auto& settings = volanim::config::settings;

    // samples directory
    settings.samples_dir = std::filesystem::path(program.get<std::string>("--samples_dir"));

    if (program.get<std::string>("--gender") == "male") {
        settings.gender = "male";
    }
    if (program.get<std::string>("--gender") == "female") {
        settings.gender = "female";
    }

    // append gender to sample dir
    if (settings.gender == "male") {
        settings.samples_dir /= "male";
    } else {
        settings.samples_dir /= "female";
    }

    // data directory
    settings.data_dir = std::filesystem::absolute(std::filesystem::path(program.get<std::string>("--data_dir")));
    if (not exists(settings.data_dir)) {
        std::cerr << "data_dir '" << settings.data_dir << "' does not exist. Exit now.\n";
        return false;
    }

    if (exists(settings.samples_dir)) {
        std::cout << "Samples dir = " <<  settings.samples_dir << std::endl;

        // check subdirs in samples dir
        if (not exists(settings.samples_dir / SAMPLES_SKELS)) {
            std::cerr << "samples_dir does not contain skel directory.\n";
            return false;
        }
        if (not exists(settings.samples_dir / SAMPLES_MUSCS)) {
            std::cerr << "samples_dir does not contain musc directory.\n";
            return false;
        }
        if (not exists(settings.samples_dir / SAMPLES_SKINS)) {
            std::cerr << "samples_dir does not contain skin directory.\n";
            return false;
        }
    } else {
        std::cout << "No samples found.\n";
    }

    // read bindpose?
    settings.load_bindpose = not program.get<bool>("--disable_bindpose");

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

auto main(const int argc, char* argv[]) -> int
{

#ifdef USE_MKL
    mkl_set_num_threads(8);
#endif

    // parse arguments
    argparse::ArgumentParser program{"CharacterViewer"};

    // argument list
    program.add_argument("--samples_dir")
        .default_value("samples")
        .help("Directory of samples of caesar fits.");

    program.add_argument("--data_dir")
        .default_value("data")
        .help("Directory of data directory.");

    program.add_argument("--gender")
        .default_value("male")
        .help("Gender of the character.");

    program.add_argument("--port")
        .default_value("8000")
        .help("Port of the network server.");

    program.add_argument("--host")
        .default_value("localhost")
        .help("Server hostname.");

    program.add_argument("--disable_bindpose")
        .default_value(false)
        .implicit_value(true)
        .help("Disable bindpose loading.");

    // parse arguments
    program.parse_args(argc, argv);
    if (not parse_cmd_line(program)) {
        std::cerr << "Could not parse config.\n";
        return 1;
    }

    auto& settings = volanim::config::settings;
    // auto& cwd = volanim::config::current_work_dir;

    // create viewer
    volanim::CharacterViewer window{};

    volumetric_base::Gender gender = volumetric_base::Gender::MALE;
    if (settings.gender == "female") {
        gender = volumetric_base::Gender::FEMALE;
    }

    window.setup_state(settings.data_dir, gender);

    // read template directory
    window.set_samples_dir(settings.samples_dir);

    // read a demo animation file
    window.read_animation(settings.data_dir / "animation" / "demo.npz");

    return window.run();
}

// ---------------------------------------------------------------------------------------------------------------------

// // =====================================================================================================================
