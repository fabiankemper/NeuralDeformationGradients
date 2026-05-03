// =====================================================================================================================
// Created by fabian on 3/26/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "layered_mesh_io.h"

#include <fstream>
#include <sstream>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(io)

// =====================================================================================================================

auto write_single_obj(FILE* file, const pmp::SurfaceMesh& mesh, const pmp::IOFlags& flags, const unsigned int vertex_offset = 0) -> void
{
    using Normal = pmp::Normal;
    using Point = pmp::Point;
    using TexCoord = pmp::TexCoord;

    // check if we can write the mesh using 32-bit indices
    const auto uint_max = std::numeric_limits<uint32_t>::max();
    if (mesh.n_vertices() > uint_max)
        throw pmp::InvalidInputException(
            "Mesh too large to be written with 32-bit indices.");

    // comment
    // fprintf(file, "# OBJ export from PMP\n");

    // write vertices
    auto points = mesh.get_vertex_property<Point>("v:point");
    for (auto v : mesh.vertices())
    {
        const Point& p = points[v];
        fprintf(file, "v %.10f %.10f %.10f\n", p[0], p[1], p[2]);
    }

    // write normals
    auto normals = mesh.get_vertex_property<Normal>("v:normal");
    if (normals && flags.use_vertex_normals)
    {
        for (auto v : mesh.vertices())
        {
            const Normal& n = normals[v];
            fprintf(file, "vn %.10f %.10f %.10f\n", n[0], n[1], n[2]);
        }
    }

    // write texture coordinates
    auto tex_coords = mesh.get_halfedge_property<TexCoord>("h:tex");
    bool write_texcoords = tex_coords && flags.use_halfedge_texcoords;

    if (write_texcoords)
    {
        if (mesh.n_halfedges() > uint_max)
            throw pmp::InvalidInputException(
                "Mesh too large to be written with 32-bit indices.");

        for (auto h : mesh.halfedges())
        {
            const TexCoord& pt = tex_coords[h];
            fprintf(file, "vt %.10f %.10f\n", pt[0], pt[1]);
        }
    }

    // write faces
    for (auto f : mesh.faces())
    {
        fprintf(file, "f");

        auto h = mesh.halfedges(f);
        for (auto v : mesh.vertices(f))
        {
            auto idx = v.idx() + 1;
            if (write_texcoords)
            {
                // write vertex index, texCoord index and normal index
                fprintf(file, " %d/%d/%d",
                    static_cast<uint32_t>(idx + vertex_offset),
                    static_cast<uint32_t>((*h).idx() + 1 + vertex_offset),
                    static_cast<uint32_t>(idx + vertex_offset)
                );
                ++h;
            }
            else
            {
                // write vertex index and normal index
                fprintf(file, " %d//%d",
                    static_cast<uint32_t>(idx + vertex_offset),
                    static_cast<uint32_t>(idx + vertex_offset)
                );
            }
        }
        fprintf(file, "\n");
    }

}

// ---------------------------------------------------------------------------------------------------------------------

auto read_single_obj(std::istream& input, unsigned int vertex_offset = 0) -> pmp::SurfaceMesh
{
    using Face = pmp::Face;
    using Point = pmp::Point;
    using TexCoord = pmp::TexCoord;
    using Vertex = pmp::Vertex;

    pmp::SurfaceMesh mesh;

    std::string line;
    std::vector<Vertex> vertices;
    std::vector<TexCoord> all_tex_coords; //individual texture coordinates
    std::vector<int> halfedge_tex_idx; //texture coordinates sorted for halfedges
    pmp::HalfedgeProperty<TexCoord> tex_coords = mesh.halfedge_property<TexCoord>("h:tex");
    bool with_tex_coord = false;


    // parse line by line (currently only supports vertex positions & faces
    while (std::getline(input, line))
    {
        // trim leading whitespace
        size_t first_char = line.find_first_not_of(" \t");
        if (first_char != std::string::npos) {
            line = line.substr(first_char);
        } else {
            line.clear(); // Skip empty lines
        }

        // Comment
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // vertex
        if (line.compare(0, 2, "v ") == 0)
        {
            std::istringstream iss(line.substr(2));
            float x, y, z;
            if (iss >> x >> y >> z)
                mesh.add_vertex(Point(x, y, z));
        }

        // fake: changed: we do not support vertex normals and texture coords
        // // normal
        // else if (strncmp(s.data(), "vn ", 3) == 0)
        // {
        //     if (sscanf(s.data(), "vn %f %f %f", &x, &y, &z))
        //     {
        //         // problematic as it can be either a vertex property when interpolated
        //         // or a halfedge property for hard edges
        //     }
        // }
        //
        // // texture coordinate
        // else if (strncmp(s.data(), "vt ", 3) == 0)
        // {
        //     if (sscanf(s.data(), "vt %f %f", &x, &y))
        //     {
        //         all_tex_coords.emplace_back(x, y);
        //     }
        // }

        // face
        else if (line.compare(0, 2, "f ") == 0)
        {
            vertices.clear();
            halfedge_tex_idx.clear();

            std::istringstream iss(line.substr(2));
            std::string token;

            while (iss >> token)
            {
                std::istringstream token_stream(token);
                std::string part;
                int v_idx = -1;
                int component = 0;

                // Split token by '/' (e.g., "v/vt/vn" or "v//vn")
                while (std::getline(token_stream, part, '/'))
                {
                    if (part.empty()) continue; // Skip empty components

                    if (component==0) {
                        v_idx = std::stoi(part);
                        if (v_idx < 0) {
                            v_idx = static_cast<int>(mesh.n_vertices()) + v_idx + 1;
                        }
                    }
                    component++;
                }

                // Add vertex (adjust for 1-based OBJ indexing)
                if (v_idx != -1) {
                    vertices.emplace_back(v_idx - 1 - vertex_offset);
                }

                // // Track texture index
                // if (vt_idx != -1)
                // {
                //     halfedge_tex_idx.push_back(vt_idx - 1 - vertex_offset);
                //     with_tex_coord = true;
                // }
            }

            // Add face
            Face f;
            try
            {
                f = mesh.add_face(vertices);
            }
            catch (const pmp::TopologyException& e)
            {
                std::cerr << e.what();
            }

            // // Assign texture coordinates
            // if (with_tex_coord && f.is_valid())
            // {
            //     auto h = mesh.halfedges(f);
            //     auto h_end = h;
            //     size_t v_idx = 0;
            //     do
            //     {
            //         tex_coords[*h] = all_tex_coords.at(halfedge_tex_idx.at(v_idx));
            //         ++v_idx;
            //         ++h;
            //     } while (h != h_end);
            // }
        }

        // // face original implementation
        // else if (line.compare(0, 2, "f ") == 0)
        // {
        //     int component(0);
        //     bool end_of_vertex(false);
        //     char *p0, *p1(s.data() + 1);
        //
        //     vertices.clear();
        //     halfedge_tex_idx.clear();
        //
        //     // skip white-spaces
        //     while (*p1 == ' ')
        //         ++p1;
        //
        //     while (p1)
        //     {
        //         p0 = p1;
        //
        //         // overwrite next separator
        //
        //         // skip '/', '\n', ' ', '\0', '\r' <-- don't forget Windows
        //         while (*p1 != '/' && *p1 != '\r' && *p1 != '\n' && *p1 != ' ' &&
        //                *p1 != '\0')
        //             ++p1;
        //
        //         // detect end of vertex
        //         if (*p1 != '/')
        //         {
        //             end_of_vertex = true;
        //         }
        //
        //         // replace separator by '\0'
        //         if (*p1 != '\0')
        //         {
        //             *p1 = '\0';
        //             p1++; // point to next token
        //         }
        //
        //         // detect end of line and break
        //         if (*p1 == '\0' || *p1 == '\n')
        //         {
        //             p1 = nullptr;
        //         }
        //
        //         // read next vertex component
        //         if (*p0 != '\0')
        //         {
        //             switch (component)
        //             {
        //                 case 0: // vertex
        //                 {
        //                     int idx = atoi(p0);
        //                     if (idx < 0)
        //                         idx = mesh.n_vertices() + idx + 1 - vertex_offset;
        //                     vertices.emplace_back(idx - 1);
        //                     break;
        //                 }
        //                 case 1: // texture coord
        //                 {
        //                     int idx = atoi(p0) - 1;
        //                     halfedge_tex_idx.push_back(idx);
        //                     with_tex_coord = true;
        //                     break;
        //                 }
        //                 case 2: // normal
        //                     break;
        //             }
        //         }
        //
        //         ++component;
        //
        //         if (end_of_vertex)
        //         {
        //             component = 0;
        //             end_of_vertex = false;
        //         }
        //     }
        //
        //     Face f;
        //     try
        //     {
        //         f = mesh.add_face(vertices);
        //     }
        //     catch (const TopologyException& e)
        //     {
        //         std::cerr << e.what();
        //     }
        //
        //     // add texture coordinates
        //     if (with_tex_coord && f.is_valid())
        //     {
        //         auto h_fit = mesh.halfedges(f);
        //         auto h_end = h_fit;
        //         unsigned v_idx = 0;
        //         do
        //         {
        //             tex_coords[*h_fit] =
        //                 all_tex_coords.at(halfedge_tex_idx.at(v_idx));
        //             ++v_idx;
        //             ++h_fit;
        //         } while (h_fit != h_end);
        //     }
        // }

    }

    // if there are no textures, delete texture property!
    if (!with_tex_coord)
    {
        mesh.remove_halfedge_property(tex_coords);
    }

    return mesh;
}


// =====================================================================================================================

auto write_layered_mesh_obj(const std::filesystem::path& filename, const LayeredMesh& layered_mesh, const pmp::IOFlags& flags) -> void
{
    // open output file for writing
    FILE* file = fopen(filename.string().c_str(), "w");
    if (!file) {
        throw pmp::IOException("Failed to open file: " + filename.string());
    }

    // comment
    fprintf(file, "# OBJ LayeredMesh Version 1\n");
    unsigned int vertex_offset = 0;

    auto write_layer = [&](const Layer& layer, const std::string& name) -> void {
        const auto mesh = layered_mesh.to_mesh(layer);
        fprintf(file, "o %s \n", name.c_str());
        write_single_obj(file, mesh, flags, vertex_offset);
        vertex_offset += static_cast<unsigned int>(mesh.n_vertices());
    };

    // write layers
    write_layer(Layer::SKIN, "skin");
    write_layer(Layer::MUSC, "musc");
    write_layer(Layer::SKEL, "skel");

    // close file
    fclose(file);
}

auto read_layered_mesh(const std::filesystem::path& filename) -> LayeredMesh
{
    std::vector<pmp::SurfaceMesh> meshes;
    std::ifstream input(filename);

    // check if file could be read
    if (!input.is_open()) {
        throw pmp::IOException("Failed to open file: " + filename.string());
    }

    // Check header line
    std::string header_line;
    if (!std::getline(input, header_line))
        throw std::runtime_error("Empty file");

    // Trim leading whitespace
    header_line.erase(0, header_line.find_first_not_of(" \t"));
    if (header_line != "# OBJ LayeredMesh Version 1")
        throw std::runtime_error("Invalid file header");

    std::vector<std::string> mesh_names;
    std::vector<std::string> expected_names = { "skin", "musc", "skel" };
    std::vector<std::string> current_group;
    unsigned int vertex_offset = 0;
    std::string line;

    while (std::getline(input, line))
    {
        // Trim leading whitespace
        line.erase(0, line.find_first_not_of(" \t"));

        // Start new group at 'o'
        if (line.rfind("o ", 0) == 0)
        {
            // Extract mesh name
            std::string name = line.substr(2);
            name.erase(0, name.find_first_not_of(" \t")); // Trim leading whitespace
            name.erase(name.find_last_not_of(" \t") + 1); // Trim trailing whitespace

            // Verify name sequence
            if (mesh_names.size() >= expected_names.size() ||
                name != expected_names[mesh_names.size()])
            {
                throw std::runtime_error("Unexpected mesh name: " + name);
            }
            mesh_names.push_back(name);

            // Process current group if not empty
            if (!current_group.empty())
            {
                std::stringstream ss;
                for (const auto& l : current_group) {
                    ss << l << "\n";
                }
                meshes.push_back(read_single_obj(ss, vertex_offset));
                vertex_offset += static_cast<unsigned int>(meshes.back().n_vertices());
                current_group.clear();
            }
        }

        // Add line to current group (skip empty lines)
        if (!line.empty()) {
            current_group.push_back(line);
        }
    }

    // Process final group
    if (!current_group.empty())
    {
        std::stringstream ss;
        for (const auto& l : current_group) {
            ss << l << "\n";
        }
        meshes.push_back(read_single_obj(ss, vertex_offset));
    }

    // check number of meshes read
    if (meshes.size() != 3) {
        throw std::runtime_error("Expected exactly 3 meshes, found " +
                               std::to_string(meshes.size()));
    }

    // build layered mesh, layers are written in reverse order
    return LayeredMesh{meshes[2], meshes[1], meshes[0] };
}

// =====================================================================================================================

NAMESPACE_END(io)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
