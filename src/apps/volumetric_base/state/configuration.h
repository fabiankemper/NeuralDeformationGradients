// =====================================================================================================================
// Created by fabian on 7/29/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// =====================================================================================================================

#include <filesystem>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)
NAMESPACE_BEGIN(config)

// =====================================================================================================================

struct Configuration {


    // directory for samples of caesar data set
    std::filesystem::path samples_dir = std::filesystem::path();

    // directory for data files
    std::filesystem::path data_dir = std::filesystem::path("data");

    // gender
    std::string gender = "male";

    // load bindpose
    bool load_bindpose = true;

// Singleton pattern (omit the extern Configuration settings; part
// usage:
//      auto& config = volanim::config::Configuration::instance();
//      config.gender = "female";
// static Configuration& instance()
// {
//     static Configuration instance;
//     return instance;
// }
// private:
//     Configuration() = default;
};

extern Configuration settings;

// =====================================================================================================================

NAMESPACE_END(config)
NAMESPACE_END(volanim)

// =====================================================================================================================

#endif //CONFIGURATION_H

// =====================================================================================================================
