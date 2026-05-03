#pragma once

#include <filesystem>

#include <pmp/mat_vec.h>

#include <pmp/bounding_box.h>

#include "SceneNode.h"

namespace pmp_character {
/// \brief Class for representing point clouds provided for example by metashape
class PointSet : public SceneNode
{
public:
    PointSet();
    PointSet(PointSet& point_set);


    void clear();

    ///  Read a point set with normals from a .xyz file.
    /// @param path
    /// @return
    bool read_xyz(const std::filesystem::path& path);

    ///  Read a point set with normals and color from a .cxyz file.
    /// @param path
    /// @return
    bool read_cxyz(const std::filesystem::path& path);

    ///  Read a point set with normals from a .obj file via pmp
    /// @param path
    /// @return
    bool read(const std::filesystem::path& path);

    /// Read a point set with normals and color from a metashape .txt file.
    /// @param path
    /// @return
    bool read_metashape_txt(const std::filesystem::path& path);

    /// Transform every point in the pointset with a given affine transformation matrix
    /// @param transform
    void transform(pmp::mat4 transform);

    /// Sets the pointcloud dirty to indicate a change in data and trigger a refresh in OpenGL
    /// @param dirty
    void set_dirty(bool dirty);

    /// Check if the pointset is marked as dirty
    /// @return
    bool is_dirty();
    /// Returns the number of points in the pointcloud depending on number of points/normals/colors
    /// @return
    size_t size() const;

    /// Calculates the bounds of the pointcloud
    /// @return
    pmp::BoundingBox bounds();

    /// The point positions
    std::vector<pmp::vec3> points;
    /// The point normals
    std::vector<pmp::vec3> normals;
    /// The point colors
    std::vector<pmp::vec3> colors;

private:
    // TODO: Use versioning unsigned integer value instead of bool?
    std::atomic<bool> dirty_;
};
}
