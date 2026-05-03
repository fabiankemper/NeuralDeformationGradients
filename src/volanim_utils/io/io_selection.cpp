// =====================================================================================================================

#include "io_selection.h"

#include <fstream>
#include <sstream>
#include <iostream>

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto read_selection(const Path& filename) -> Selection
{
    Selection result;
    std::ifstream ifstream(filename);
    if (!ifstream) {
        std::cerr << "[ERROR] Could not open " << filename.string() << " for reading index selection.\n";
        return result;
    }

    std::string line;
    while (std::getline(ifstream, line))
    {
        unsigned int index;
        std::stringstream stringstream(line);
        stringstream >> index;

        if (!stringstream.fail()) {
            result.push_back(index);
        }
    }
    ifstream.close();

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_selection(const Selection& selection, const Path& filename) -> bool
{
    std::ofstream ofstream(filename);
    if (!ofstream) {
        std::cerr << "[ERROR] Could not open " << filename.string() << " for writing.\n";
        return false;
    }

    for (const auto& index : selection) {
        ofstream << index << "\n";
    }

    ofstream.close();
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================