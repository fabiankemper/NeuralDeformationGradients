#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>

#include <Eigen/Core>
#include <npy.hpp>

#include "volumetric_base/layered_mesh.h"

namespace util::io{
    inline void saveMatrixToBinary(const Eigen::MatrixXd& matrix, const std::filesystem::path& filename) {
        std::ofstream file(filename, std::ios::binary);
        if (file.is_open()) {
            int rows = static_cast<int>(matrix.rows());
            int cols = static_cast<int>(matrix.cols());
            file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
            file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
            file.write(reinterpret_cast<const char*>(matrix.data()), static_cast<long>(rows * cols * sizeof(double)));
            file.close();
        } else {
            std::cerr << "Could not open file for writing: " << filename.string() << std::endl;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------


    inline Eigen::MatrixXd readTetgenNodeFile(const std::filesystem::path& filename) {
        std::ifstream infile(filename);
        if (!infile) {
            throw std::runtime_error("Cannot open file: " + filename.string());
        }

        std::string line;
        // Skip comment lines
        while (std::getline(infile, line)) {
            if (line.empty() || line[0] == '#') continue;
            break;
        }

        std::istringstream header(line);
        int numPoints, dim, numAttr, hasMarker;
        header >> numPoints >> dim >> numAttr >> hasMarker;

        Eigen::MatrixXd V(numPoints, dim);
        for (int i = 0; i < numPoints; ++i) {
            int idx;
            double x, y, z;
            infile >> idx >> x >> y >> z;
            V(i, 0) = x;
            if (dim > 1) V(i, 1) = y;
            if (dim > 2) V(i, 2) = z;
        }

        return V;
    }
//----------------------------------------------------------------------------------------------------------------------

    inline Eigen::MatrixXd loadMatrixFromBinary(const std::filesystem::path& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (file.is_open()) {
            int rows = 0, cols = 0;
            file.read(reinterpret_cast<char*>(&rows), sizeof(int));
            file.read(reinterpret_cast<char*>(&cols), sizeof(int));
            Eigen::MatrixXd matrix(rows, cols);
            file.read(reinterpret_cast<char*>(matrix.data()), static_cast<long>(rows * cols * sizeof(double)));
            std::cout << rows  << cols << std::endl;
            file.close();
            return matrix;
        } else {
            std::cerr << "Could not open file for reading: " << filename << std::endl;
            return {};
        }
    }

//----------------------------------------------------------------------------------------------------------------------

    inline std::vector<std::filesystem::path> getAllFilesWithExtension(const std::filesystem::path& path, const std::string& extension)
    {
        std::vector<std::filesystem::path> result;
        try{
        for(const auto & entry : std::filesystem::recursive_directory_iterator(path))
        {
            if(entry.is_regular_file())
            {
                if(extension == entry.path().extension().string())
                {
                    result.emplace_back(entry.path());
                }
            }
        }
        }
        catch (const std::filesystem::filesystem_error &e)
        {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
        }
        return result;
    }

//======================================================================================================================

    inline void store_as_npy(const Eigen::MatrixXd& matrix, const std::filesystem::path& filename) {
        npy::npy_data<double> data;
        data.shape = {
            static_cast<unsigned long>(matrix.rows()),
            static_cast<unsigned long>(matrix.cols()) \
        };
        data.data = std::vector<double>(matrix.cols() * matrix.rows());
        for(int i=0; i<matrix.rows(); i++) {
            for(int j=0; j<matrix.cols(); j++) {
                data.data[i * matrix.cols() + j] = matrix(i, j);
            }
        }
        write_npy(filename.string(), data);
    }

    //----------------------------------------------------------------------------------------------------------------------

    inline void store_as_npyI(const Eigen::MatrixXi& matrix, const std::filesystem::path& filename) {
        npy::npy_data<int> data;
        data.shape = {
            static_cast<unsigned long>(matrix.rows()),
            static_cast<unsigned long>(matrix.cols()) \
        };
        data.data = std::vector<int>(matrix.cols() * matrix.rows());
        for(int i=0; i<matrix.rows(); i++) {
            for(int j=0; j<matrix.cols(); j++) {
                data.data[i * matrix.cols() + j] = matrix(i, j);
            }
        }
        write_npy(filename.string(), data);
    }

//----------------------------------------------------------------------------------------------------------------------

    // TODO: REPLACE BY volanim_utils/npy_utils.h
    inline Eigen::MatrixXd load_as_npy(const std::filesystem::path& filename) {
        const npy::npy_data<double> data = npy::read_npy<double>(filename.string());
        assert(data.shape.size() == 2);
        Eigen::MatrixXd result(data.shape[0], data.shape[1]);
        for(unsigned int i=0; i<data.shape[0]; i++) {
            for(unsigned int j=0; j<data.shape[1]; j++) {
                result(i, j) = data.data[i * data.shape[1] + j];
            }
        }
        return result;
    }

    //----------------------------------------------------------------------------------------------------------------------

    inline std::vector<std::array<int,4>> readTetsFromFile(const std::filesystem::path& filename) {
        std::vector<std::array<int,4>> tets;
        std::ifstream infile(filename);
        if (!infile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return tets;
        }

        std::string line;

        // Skip the first line
        std::getline(infile, line);

        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            int tetIndex, v0, v1, v2, v3;
            if (iss >> tetIndex >> v0 >> v1 >> v2 >> v3) {
                tets.push_back({v0, v1, v2, v3});
            }
        }

        return tets;
    }
//----------------------------------------------------------------------------------------------------------------------

    inline Eigen::MatrixXi load_as_npyI(const std::filesystem::path& filename) {
        npy::npy_data<int> data = npy::read_npy<int>(filename.string());
        assert(data.shape.size() == 2);
        Eigen::MatrixXi result(data.shape[0], data.shape[1]);
        for(unsigned int i=0; i<data.shape[0]; i++) {
            for(unsigned int j=0; j<data.shape[1]; j++) {
                result(i, j) = data.data[i * data.shape[1] + j];
            }
        }
        return result;
    }

//======================================================================================================================


    inline void get_possible_mesh_indices(const std::filesystem::path& path, const std::string& reststring, std::vector<int> &numbers) {
        std::regex regex("(\\d+)"+reststring);

        std::smatch match;
        for( auto & entry : std::filesystem::directory_iterator(path)) {
            std::string filename = entry.path().filename().string();
            if(std::regex_search(filename, match, regex)) {
                //remove roughly 1/10 of bodies
                if ((stoi(match[1])%10) != 9)
                numbers.emplace_back(stoi(match[1]));
            }
            else {
                std::cout << "Could not regex parse " << filename << std::endl;
            }
        }
    }


    inline void get_possible_validation_mesh_indices(const std::filesystem::path& path, const std::string& reststring, std::vector<int> &numbers) {
        std::regex regex("(\\d+)"+reststring);

        std::smatch match;
        for( auto & entry : std::filesystem::directory_iterator(path)) {
            std::string filename = entry.path().filename().string();
            if(std::regex_search(filename, match, regex)) {
                if ((stoi(match[1])%10) == 9)
                    numbers.emplace_back(stoi(match[1]));
            }
            else {
                std::cout << "Could not regex parse " << filename << std::endl;
            }
        }
    }


    inline void get_possible_test_mesh_indices(const std::filesystem::path& path, const std::string& reststring, std::vector<int> &numbers) {
        std::regex regex("(\\d+)"+reststring);

        std::smatch match;
        for( auto & entry : std::filesystem::directory_iterator(path)) {
            std::string filename = entry.path().filename().string();
            if(std::regex_search(filename, match, regex)) {
                if ((stoi(match[1])%10) == 8)
                    numbers.emplace_back(stoi(match[1]));
            }
            else {
                std::cout << "Could not regex parse " << filename << std::endl;
            }
        }
    }

//----------------------------------------------------------------------------------------------------------------------

    inline void get_possible_mesh_indices(const std::filesystem::path& path, const std::string& reststring, const std::string& path_delim, std::map<int, std::filesystem::path> &file_map) {
        std::regex regex("(\\d+)" + reststring);

        std::smatch match;
        // Use recursive_directory_iterator for recursive traversal
        for (auto & entry : std::filesystem::recursive_directory_iterator(path)) {
            // Check if the entry is a regular file (not a directory)
            if (std::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                if (std::regex_search(filename, match, regex)) {
                    // Convert the matched string to an integer (index)
                    int index = std::stoi(match[1]);

                    // Remove roughly 1/10 of bodies
                    if ((index % 10) != 9 && index % 10 != 8){
                        std::string full_path = entry.path().string();
                        size_t pos = full_path.find(path_delim);
                        file_map[index] = full_path.substr(0,pos);
                    }
                }

            }
        }
    }


    inline void get_possible_val_mesh_indices(const std::filesystem::path& path, const std::string& reststring, const std::string& path_delim, std::map<int, std::filesystem::path> &file_map) {
        std::regex regex("(\\d+)" + reststring);

        std::smatch match;
        // Use recursive_directory_iterator for recursive traversal
        for (auto & entry : std::filesystem::recursive_directory_iterator(path)) {
            // Check if the entry is a regular file (not a directory)
            if (std::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                if (std::regex_search(filename, match, regex)) {
                    // Convert the matched string to an integer (index)
                    int index = std::stoi(match[1]);
                    if ((index % 10) == 8){
                        std::string full_path = entry.path().string();
                        size_t pos = full_path.find(path_delim);
                        file_map[index] = full_path.substr(0,pos);
                    }
                }

            }
        }
    }

    inline void get_possible_test_mesh_indices(const std::filesystem::path& path, const std::string& reststring, const std::string& path_delim, std::map<int, std::filesystem::path> &file_map) {
        std::regex regex("(\\d+)" + reststring);

        std::smatch match;
        // Use recursive_directory_iterator for recursive traversal
        for (auto & entry : std::filesystem::recursive_directory_iterator(path)) {
            // Check if the entry is a regular file (not a directory)
            if (std::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                if (std::regex_search(filename, match, regex)) {
                    // Convert the matched string to an integer (index)
                    int index = std::stoi(match[1]);
                    if ((index % 10) == 9){
                        std::string full_path = entry.path().string();
                        size_t pos = full_path.find(path_delim);
                        file_map[index] = full_path.substr(0,pos);
                    }
                }

            }
        }
    }
//----------------------------------------------------------------------------------------------------------------------

    inline volumetric_base::LayeredMesh get_layered_mesh(const std::filesystem::path& path, int i) {
        volumetric_base::LayeredMesh skel, musc, skin;
        pmp::read(skel, path/"skel"/(std::to_string(i)+"_skel.off"));
        pmp::read(musc, path/"musc_max"/(std::to_string(i)+"_musc_max.off"));
        pmp::read(skin, path/"skin"/(std::to_string(i)+"_skin.off"));
        return {skel,musc,skin};
    }
}
