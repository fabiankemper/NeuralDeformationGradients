#include "PointSet.h"

#include <fstream>
#include <pmp/surface_mesh.h>

#include <pmp/io/io.h>
#include <pmp_character/PropertyNames.h>

// TODO(swenninger): Namespacing

namespace pmp_character
{
    PointSet::PointSet() : SceneNode("PointCloud")
    {
        nodeType = NodeType::PointSet;
    }

    PointSet::PointSet(PointSet& point_set) : SceneNode("PointCloud")
    {
        nodeType = NodeType::PointSet;
        for(auto v : point_set.points)
        {
            points.emplace_back(v);
        }
        for(auto n : point_set.normals)
        {
            normals.emplace_back(n);
        }
        for(auto c : point_set.colors)
        {
            colors.emplace_back(c);
        }
    }


    void PointSet::clear()
    {
        points.clear();
        normals.clear();
        colors.clear();
    }

    bool PointSet::read_xyz(const std::filesystem::path& path)
    {
        // Format per line:
        //      x y z nx ny nz

        clear();

        std::ifstream ifs(path);

        if (!ifs)
        {
            return false;
        }

        pmp::vec3 p;
        pmp::vec3 n;

        std::string line;
        while (std::getline(ifs, line))    {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            std::stringstream ss(line);

            ss >> p[0] >> p[1] >> p[2];
            ss >> n[0] >> n[1] >> n[2];

            points.push_back(p);
            n.normalize();
            normals.push_back(n);
        }

        return true;
    }

    bool PointSet::read_cxyz(const std::filesystem::path& path)
    {
        // Format per line:
        //      x y z nx ny nz cx cy cz
        // with cx,cy,cz in [0 - 255]

        clear();

        std::ifstream ifs(path);

        if (!ifs)
        {
            return false;
        }

        pmp::vec3 p;
        pmp::vec3 n;
        pmp::vec3 c;

        while (!ifs.eof())
        {
            ifs >> p[0] >> p[1] >> p[2];
            ifs >> n[0] >> n[1] >> n[2];
            ifs >> c[0] >> c[1] >> c[2];

            n.normalize();
            c /= 255.0;

            points.push_back(p);
            normals.push_back(n);
            colors.push_back(c);
        }

        return true;
    }

    bool PointSet::read(const std::filesystem::path& path)
    {
        if(exists(path) and is_regular_file(path))
        {
            pmp::SurfaceMesh mesh;
            pmp::read(mesh, path);
            points.reserve(mesh.n_vertices());
            normals.reserve(mesh.n_vertices());
            colors.reserve(mesh.n_vertices());
            auto meshColor = mesh.vertex_property<pmp::Color>(pmp_character::properties::vertexColor);
            auto normal = mesh.vertex_property<pmp::Normal>(pmp_character::properties::vertexNormals);
            for(size_t i=0; i < mesh.n_vertices(); i++)
            {
                const auto v = pmp::Vertex(i);
                points[i] = mesh.position(v);
                normals[i] = normal[v];
                colors[i] = meshColor[v];
            }
            return true;
        }
        return false;
    }

    bool PointSet::read_metashape_txt(const std::filesystem::path& path)
    {
        // Format per line:
        //      x y z cx cy cz nx ny nz
        // with cx,cy,cz in [0 - 255]

        clear();

        std::ifstream ifs(path);

        if (!ifs)
        {
            return false;
        }


        pmp::vec3 p;
        pmp::vec3 n;
        pmp::vec3 c;

        while (!ifs.eof())
        {
            ifs >> p[0] >> p[1] >> p[2];
            ifs >> c[0] >> c[1] >> c[2];
            ifs >> n[0] >> n[1] >> n[2];

            n.normalize();
            c /= 255.0;

            points.push_back(p);
            normals.push_back(n);
            colors.push_back(c);
        }

        return true;
    }

    void PointSet::transform(pmp::mat4 transform)
    {
        // First transform all points
#pragma omp parallel for
        for(long i = 0; i < points.size(); i++)
        {
            points[i] = pmp::affine_transform(transform,points[i]);
        }
        // Transform the normals
        pmp::mat4 normalTransform = transpose(inverse(transform));
#pragma omp parallel for
        for(long i = 0; i < normals.size(); i++)
        {
            normals[i] = pmp::affine_transform(normalTransform,normals[i]);
        }
        set_dirty(true);
    }

    void PointSet::set_dirty(bool dirty)
    {
        dirty_.store(dirty);
    }

    bool PointSet::is_dirty()
    {
        return dirty_.load();
    }

    size_t PointSet::size() const
    {
        return points.size();
    }

    pmp::BoundingBox PointSet::bounds()
    {
        pmp::BoundingBox bb;
        for (auto v : points)
            bb += v;
        return bb;
    }
}