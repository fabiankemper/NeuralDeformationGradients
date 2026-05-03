//
// Created by fabian on 7/10/24.
//
// =====================================================================================================================

#pragma once
#ifndef IO_NPZ_H
#define IO_NPZ_H

// =====================================================================================================================

#include "volanim_utils/defs.h"

#include <filesystem>
#include <set>
#include <streambuf>
#include <type_traits>

#include <npy.hpp>
#include <libzippp.h>

#include "memory_buffer.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

using Path = std::filesystem::path;
using ZipArchive = libzippp::ZipArchive;
using ZipEntry = libzippp::ZipEntry;
using ZipOpenMode = libzippp::ZipArchive::OpenMode;

// =====================================================================================================================

class NPZFile {
public:
    NPZFile() = default;
    virtual ~NPZFile();
    explicit NPZFile(const Path& filename, const ZipOpenMode& mode = ZipOpenMode::ReadOnly);

    // get entries
    [[nodiscard]]
    auto entries() const -> std::set<std::string>;
    [[nodiscard]]
    auto contains(const std::string& entry) const -> bool;

    // get entry
    template<typename T>
    auto get_entry(const std::string& entry_name) -> npy::npy_data<T>;

    // add data entry
    template<typename T>
    auto add_entry(const std::string& entry_name, const npy::npy_data<T>& data) -> void;
    template<typename T>
    auto add_entry(const std::string& entry_name, const npy::npy_data_ptr<T>& data) -> void;

    // remove entry
    auto remove_entry(const std::string& entry_name) -> void;

    // commit and save to file
    auto write() const -> void;
    auto rollback() const -> void;

protected:
    // pointer to zip archive file
    ZipArchive* _zip_archive = nullptr;

    // current file mode
    ZipOpenMode _mode = ZipOpenMode::ReadOnly;

    // made changes to current file
    bool _dirty = false;

    // expand entry name
    static auto expand_entry_name(const std::string& entry_name) -> std::string;
};

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

template<typename T>
auto NPZFile::get_entry(const std::string& entry_name) -> npy::npy_data<T>
{
    if (not _zip_archive) {
        return {};
    }

    // append extension for numpy files
    const auto entry_ext = expand_entry_name(entry_name);
    const ZipEntry entry = _zip_archive->getEntry(entry_ext);

    if (entry.isNull()) {
        std::string error_message = "Could not find entry " + entry_name + " in archive.";
        throw std::runtime_error(error_message);
    }

    std::stringstream buffer;
    // buffer.str(std::string()); // clear buffer
    entry.readContent(buffer, ZipArchive::State::Original);

    // reset reading head
    buffer.seekp(0, std::ios_base::beg);

    // read data
    try {
        if constexpr (std::is_same_v<T, std::string>) {
            npy::npy_data npy_data = npy::read_npy_string(buffer);
            return npy_data;
        } else {
            npy::npy_data npy_data = npy::read_npy<T>(buffer);
            return npy_data;
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Could not read " + entry_name + " from archive. Incorrect numpy datatype (e.g. float != f8) was given.");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

template<typename T>
auto NPZFile::add_entry(const std::string& entry_name, const npy::npy_data<T>& data) -> void
{
    if (contains(entry_name)) {
        throw std::runtime_error("NPZ already contains an entry " + entry_name + ".");
    }

    MemoryBuffer buffer;
    std::ostream ostream(&buffer);

    // write data
    if constexpr (std::is_same_v<T, std::string>) {
        npy::write_npy_string(ostream, data);
    } else {
        npy::write_npy(ostream, data);
    }

    // create persistent memory region
    const auto o_buffer = new char[buffer.size()];
    std::memcpy(o_buffer, buffer.data(), buffer.size());

    // internal name with extension
    const auto entry_ext = expand_entry_name(entry_name);

    // create new entry
    auto entry = _zip_archive->addData(entry_ext, o_buffer, buffer.size(), /*freeData=*/true);
    if (not entry) {
        std::cerr << "Could not add entry " << entry_name << " to npz file.\n";
    } else {
        _dirty = true;
    }

    // delete[] o_buffer;
}

// ---------------------------------------------------------------------------------------------------------------------

template<typename T>
auto NPZFile::add_entry(const std::string& entry_name, const npy::npy_data_ptr<T>& data) -> void
{
    if (contains(entry_name)) {
        throw std::runtime_error("NPZ already contains an entry " + entry_name + ".");
    }

    MemoryBuffer buffer;
    std::ostream ostream(&buffer);

    // write data
    npy::write_npy(ostream, data);

    // create persistent memory region
    char* o_buffer = new char[buffer.size()];
    std::memcpy(o_buffer, buffer.data(), buffer.size());

    // internal name with extension
    auto entry_ext = expand_entry_name(entry_name);

    // create new entry
    auto entry = _zip_archive->addData(entry_ext, o_buffer, buffer.size(), /*freeData=*/true);
    if (not entry) {
        std::cerr << "Could not add entry " << entry_name << " to npz file.\n";
    } else {
        _dirty = true;
    }

    // delete[] o_buffer;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================

#endif //IO_NPZ_H

// =====================================================================================================================
