//
// Created by fabian on 7/10/24.
//
// =====================================================================================================================

#include "io_npz.h"

#include "volanim_utils/string_utils.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

NPZFile::~NPZFile()
{
    if (_zip_archive) {
        _zip_archive->close();
    }
    delete _zip_archive;
}

// ---------------------------------------------------------------------------------------------------------------------

NPZFile::NPZFile(const Path& filename, const ZipOpenMode& mode)
{
    if (not exists(filename) and mode == ZipOpenMode::ReadOnly) {
        throw std::runtime_error("Could not open " + filename.string() + ". File not found.");
    }

    // create internal zip file buffer
    _zip_archive = new ZipArchive(filename.string());

    // open archive
    if (not _zip_archive->open(mode)) {
        std::cerr << "Could not open NPZ file: " << filename.string() << ".\n";
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::entries() const -> std::set<std::string>
{
    auto entries_ = _zip_archive->getEntries(ZipArchive::State::Current);
    std::set<std::string> result;

    for (auto& entry : entries_) {
        std::string fname = entry.getName();
        if (volanim::utils::string_has_ending(fname, ".npy")) {
            result.emplace(fname.substr(0, fname.length() - 4));
        }
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::contains(const std::string& entry) const -> bool
{
    auto entries_ = entries();
    return entries_.contains(entry);
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::remove_entry(const std::string& entry_name) -> void
{
    // do not remove if not contained
    if (not contains(entry_name)) {
        return;
    }

    // add extension for npz
    auto entry_ext = expand_entry_name(entry_name);
    auto result = _zip_archive->deleteEntry(entry_ext);
    _dirty = true;

    if (result < 0) {
        throw std::runtime_error("Could not remove entry " + entry_name + " from zip. Reason: " + std::to_string(result) + ".");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::write() const -> void
{
    if (_zip_archive) {
        _zip_archive->close();
        // _zip_archive->open(_mode);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::rollback() const -> void
{
    if (_zip_archive) {
        _zip_archive->discard();
        _zip_archive->open(_mode);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto NPZFile::expand_entry_name(const std::string& entry_name) -> std::string
{
    return entry_name + ".npy";
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================
