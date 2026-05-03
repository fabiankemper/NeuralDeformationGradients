// =====================================================================================================================
// Created by fabian on 9/11/24.
// mail@fabiankemper.de
// =====================================================================================================================

#include "animation.h"

#include <npy.hpp>
#include <ranges>

#include "volanim_utils/string_utils.h"
#include "volanim_utils/io/io_npz.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(animation)

// =====================================================================================================================

using NPZFile = volanim::utils::NPZFile;

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------

auto Animation::from_numpy(const Path& filename) -> void
{
    if (not exists(filename)) {
        std::cerr << "Could not find pose data file " << filename << "\n";
        return;
    }

    npy::npy_data poses_npy = npy::read_npy<double>(filename.string());

    _poses_buffer.reserve(poses_npy.data.size());
    _poses_buffer.resize(poses_npy.data.size());

    // copy frame data
    for (size_t index = 0; index < poses_npy.data.size(); ++index) {
        _poses_buffer[index] = static_cast<float>(poses_npy.data[index]);
    }

    std::vector<unsigned long> shape = poses_npy.shape;
    bool fortran_order = poses_npy.fortran_order;

    std::cout << "Found " << poses_npy.data.size() << " entries data in npy.\n";
    std::cout << "Shape ";
    for (auto& dim : shape) {
        std::cout << dim << ", ";
    }
    std::cout << "\n";
    std::cout << "Is fortran order " << fortran_order << "\n";

    if (shape.size() < 2) {
        std::cerr << "Loading pose failed. Incorrect number of dimensions in pose file.\n";
        return;
    }

    _framerate = 60.0f;
    _num_pose_parameters = static_cast<int>(shape[1]);
    _total_frames = static_cast<int>(shape[0]);
    _current_frame = 0;
}

// ---------------------------------------------------------------------------------------------------------------------

auto _read_numpyz_double(NPZFile& npz_file, const std::string& entry) -> npy::npy_data<float>
{
    // read buffers
    auto data = npz_file.get_entry<double>(entry);

    // prepare buffer
    auto data_converted = npy::npy_data<float>();
    data_converted.shape = data.shape;
    data_converted.fortran_order = data.fortran_order;

    // get number of elements
    size_t total_elements = 1;
    for (const auto& dim : data.shape) {
        total_elements *= dim;
    }
    // resize buffer based on total elements
    data_converted.data.resize(total_elements);

    // fill buffer
    for (size_t index = 0; index < total_elements; ++index) {
        data_converted.data[index] = static_cast<float>(data.data[index]);
    }

    return data_converted;
}

// ---------------------------------------------------------------------------------------------------------------------

auto _read_numpyz_float(NPZFile& npz_file, const std::string& entry) -> npy::npy_data<float>
{
    auto data = npz_file.get_entry<float>(entry);
    return data;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::from_numpyz(
    const Path& filename,
    const std::string& pose_entry,
    const std::string& trans_entry,
    const std::string& framerate_entry
) -> void
{
    if (not exists(filename)) {
        std::cerr << "Could not find pose data file " << filename << "\n";
        return;
    }

    auto npz_file = NPZFile(filename);

    if (not npz_file.contains(pose_entry)  or not npz_file.contains(trans_entry) or not npz_file.contains(framerate_entry)) {
        std::cerr << "NPZ format is not a valid pose sequence.\n";
        return;
    }

    int64_t version = 1;

    if (npz_file.contains("version")) {
        const auto version_data = npz_file.get_entry<int64_t>("version");
        version = version_data.data[0];
    }

    // read buffers
    npy::npy_data<float> poses, trans, framerate;
    if (version == 2) {
        poses = _read_numpyz_double(npz_file, pose_entry);
        trans = _read_numpyz_double(npz_file, trans_entry);
        framerate = _read_numpyz_double(npz_file, framerate_entry);
    } else if (version == 3) {
        poses = _read_numpyz_float(npz_file, pose_entry);
        trans = _read_numpyz_float(npz_file, trans_entry);
        framerate = _read_numpyz_float(npz_file, framerate_entry);
    } else {
        std::cerr << "Unsupported version number (" << version << ")\n";
        return;
    }

    // check preconditions
    if (poses.shape.size() != 2) {
        std::cerr << "Poses dimension(" << poses.shape.size() << ") does match expected dimensions (2).\n";
        return;
    }
    if (trans.shape.size() != 2) {
        std::cerr << "Trans dimension(" << trans.shape.size() << ") does match expected dimensions (2).\n";
        return;
    }
    if (trans.shape[1] != 3) {
        std::cerr << "Trans dimension(1) has shape " << trans.shape[1] << ", expected 3.\n";
        return;
    }

    // process fps
    _framerate = static_cast<float>(framerate.data[0]);

    // process poses
    const auto pose_buffer_size = poses.shape[0] * poses.shape[1];
    _poses_buffer.resize(pose_buffer_size);

    for (size_t index = 0; index < pose_buffer_size; ++index) {
        _poses_buffer[index] = static_cast<float>(poses.data[index]);
    }

    // process trans
    const auto trans_buffer_size = trans.shape[0] * trans.shape[1];
    _trans_buffer.resize(trans_buffer_size);

    for (size_t index = 0; index < trans_buffer_size; ++index) {
        _trans_buffer[index] = static_cast<float>(trans.data[index]);
    }

    _current_frame = 0;
    _num_pose_parameters = static_cast<int>(poses.shape[1]);
    _total_frames = static_cast<int>(poses.shape[0]);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::from_csv(const Path& filename) -> void
{
    // check existence
    if (not exists(filename)) {
        std::cerr << "Could not find pose data file " << filename << "\n";
        return;
    }

    // open file for reading
    std::ifstream file(filename);
    if (not file.is_open()) {
        std::cerr << "Could not open file " << filename << "\n";
        return;
    }

    // --- helper methods ---
    auto is_comment_line = [&](const std::string& line) -> bool {
        return not line.empty() && line[0] == '#';
    };

    auto split = [&](const std::string& line, const char delimiter) -> std::vector<std::string> {
        std::vector<std::string> result;
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, delimiter)) {
            result.push_back(token);
        }
        return result;
    };

    // line buffer
    std::string line;

    // check for metadata lines
    while (std::getline(file, line)) {
        if (is_comment_line(line)) {
            if (line.rfind("# framerate:", 0) == 0) {
                _framerate = std::stof(line.substr(12));
                std::cout << "Framerate " << _framerate << std::endl;
            }
            if (line.rfind("# version:", 0) == 0) {
                if (const long version = std::stol(line.substr(10)); version != 2) {
                    std::cerr << "Invalid version number.\n";
                }
            }
        } else {
            break; // break on first non-comment line
        }
    }

    auto headers = std::vector<std::string>();

    // check for optional column headers
    if (not line.empty() && std::isalpha(line[0])) {
        headers = split(line, ',');
        if (std::getline(file, line).fail()) {
            throw std::runtime_error("No data rows after column headers");
        }
    }

    // save rows
    auto data_rows = std::vector<std::vector<float>>();

    // parse the data rows
    do {
        auto values = split(line, ',');

        std::vector<float> row;
        for (const auto& value : values) {
            row.push_back(std::stof(value));
        }

        // consistency check: data row size should be 3 times header size (if header is given)
        if (not headers.empty() && row.size() != headers.size() * 3) {
            std::cerr << "Incorrect number of headers (" << row.size() << ") in file " << filename << ". Expected " << headers.size() * 3 << "\n";
            return;
        }
        // add to row buffer
        data_rows.push_back(row);
    } while (std::getline(file, line));

    if (data_rows.empty()) {
        std::cerr << "No data rows in file " << filename << "\n";
        return;
    }

    // reserve buffers
    long joint_cnt = (static_cast<long>(data_rows[0].size()) / 3) - 1;
    if (joint_cnt < 0) {
        std::cerr << "Invalid joint count in " << filename << "\n";
        return;
    }

    _trans_buffer.clear();
    _poses_buffer.clear();
    _trans_buffer.resize(data_rows.size() * 3);
    _poses_buffer.resize(data_rows.size() * (joint_cnt * 3));

    // set frame number
    _current_frame = 0;
    _total_frames = static_cast<long>(data_rows.size());

    // convert to frames
    long frame = 0;

    for (const auto& row : data_rows) {
        if (row.size() == static_cast<size_t>(joint_cnt * 3 + 3)) {
            // no specific order given
            _trans_buffer[frame * 3 + 0] = static_cast<float>(row[0]);
            _trans_buffer[frame * 3 + 1] = static_cast<float>(row[1]);
            _trans_buffer[frame * 3 + 2] = static_cast<float>(row[2]);

            // todo: check header order

            // move pose buffer
            std::memmove(_poses_buffer.data() + joint_cnt * 3 * frame, row.data() + 3, 3 * joint_cnt * sizeof(float));
            // each entry separately
            // for (long pose_idx = 0; pose_idx < joint_cnt * 3; ++pose_idx) {
            //     _poses_buffer[frame * joint_cnt * 3 + pose_idx] = static_cast<float>(row[pose_idx]);
            // }

            // next frame
            frame++;
        } else {
            std::cerr << "Frame " << frame << " does not have correct format.\n";
        }
    }

    file.close();
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::from_posefile(const Path& filename) -> void
{
    _total_frames = 1;
    _current_frame = 0;
    _framerate = 120.0f; // this does not matter, as we only have a single frame

    _poses_buffer.clear();
    _trans_buffer.clear();

    // read pose angles
    const npy::npy_data<double> data = npy::read_npy<double>(filename.string());
    assert(data.shape.size() == 2);
    Eigen::MatrixXd pose_data(data.shape[0], data.shape[1]);
    for(unsigned int i=0; i<data.shape[0]; i++) {
        for(unsigned int j=0; j<data.shape[1]; j++) {
            pose_data(i, j) = data.data[i * data.shape[1] + j];
        }
    }

    // copy pose to internal buffer
    _poses_buffer.resize(pose_data.size());
    for (size_t index = 0; index < _poses_buffer.size(); index++) {
        _poses_buffer[index] = static_cast<float>(pose_data.data()[index]);
    }

    // no translation support in this file
    _trans_buffer.resize(3);
    std::ranges::fill(_trans_buffer, 0.0f);
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::framerate() const -> double
{
    return _framerate;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::current_frame() const -> long
{
    return _current_frame;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::total_frames() const -> long
{
    return _total_frames;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::set_frame(long frame) -> void
{
    // clamp
    if (frame < 0) {
        frame = 0;
    }
    if (frame >= _total_frames) {
        frame = _total_frames - 1;
    }
    _current_frame = frame;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::step_frames(const long frames) -> void
{
    const auto next_frame = _current_frame + frames;
    if ((next_frame < 0) || (next_frame >= _total_frames)) {
        // rollback to start/end
        set_frame(next_frame % _total_frames);
    }
    else {
        set_frame(next_frame);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::current_pose() const -> std::vector<float>
{
    if (_poses_buffer.empty() || _poses_buffer.size() < static_cast<size_t>(_num_pose_parameters)) {
        std::vector<float> result;
        result.resize(_num_pose_parameters);
        return result;
    }

    // // std::vector<float>::const_iterator
    const auto first = _poses_buffer.cbegin() + _current_frame * _num_pose_parameters;
    const auto last = _poses_buffer.cbegin() + (_current_frame + 1) * _num_pose_parameters;

    std::vector<float> result(first, last);
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::current_trans() const -> std::array<float, 3>
{
    if (_poses_buffer.empty() || (static_cast<long>(_poses_buffer.size()) < _num_pose_parameters)) {
        return { 0.0f, 0.0f, 0.0f };
    }

    const auto first = _trans_buffer.cbegin() + _current_frame * _num_trans_parameters;
    return {first[0], first[1], first[2] };
}

// ---------------------------------------------------------------------------------------------------------------------

auto Animation::read(std::filesystem::path filename) -> void
{
    if (not exists(filename)) {
        throw std::runtime_error("File '" + filename.string() + "' does not exist");
    }
    if (filename.extension() == ".npz") {
        from_numpyz(filename);
    } else if (filename.extension() == ".npy") {
        from_numpy(filename);
    } else if (filename.extension() == ".csv") {
        from_csv(filename);
    } else if (filename.extension() == ".pose") {
        from_posefile(filename);
    } else {
        std::cerr << "File " << filename << " coult not be read (invalid format).\n";
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(animation)

NAMESPACE_END(volumetric_base)

// =====================================================================================================================
