// =====================================================================================================================

#include "pmp_ext/pmp_io.h"

#include <pmp/io/io.h>

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto write_mesh_pos_property(const SurfaceMesh& mesh, const Path& filename, const String& property_name) -> void
{
    if (not mesh.has_vertex_property(property_name)) {
        throw std::runtime_error("Could not find property " + property_name + " on mesh.");
    }

    pmp::SurfaceMesh out_mesh;
    // copy connectivity
    out_mesh.assign(mesh);

    auto pos_prop = mesh.get_vertex_property<pmp::Point>(property_name);
    // assign vertex positions
    for (auto vertex : mesh.vertices()) {
        out_mesh.position(vertex) = pos_prop[vertex];
    }

    // write mesh
    pmp::write(out_mesh, filename);
}

// ---------------------------------------------------------------------------------------------------------------------

auto trim_float(const double val, int precision = 7) -> std::string
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.*f", precision, val);
    // remove trailing zeros
    char* end = buf + strlen(buf) - 1;
    while (end > buf && *end == '0' && *(end-1) != '.') {
        *end-- = '\0';
    }
    return {buf};
}

// ---------------------------------------------------------------------------------------------------------------------

auto write_small_obj(const SurfaceMesh& mesh, const std::filesystem::path& filename, IOFlags io_flags) -> void
{
    FILE* out = fopen(filename.string().c_str(), "w");
    if (!out) {
        throw pmp::IOException("Failed to open file: " + filename.string());
    }

    // check if we can write the mesh using 32-bit indices
    const auto uint_max = std::numeric_limits<uint32_t>::max();
    if (mesh.n_vertices() > uint_max) {
        throw pmp::InvalidInputException(
            "Mesh too large to be written with 32-bit indices.");
    }

    // comment
    fprintf(out, "# Small - OBJ export from PMP\n");

    // write vertices
    auto points = mesh.get_vertex_property<pmp::Point>("v:point");
    for (auto v : mesh.vertices())
    {
        const pmp::Point& p = points[v];
        fprintf(out, "v %.7f %.7f %.7f\n", p[0], p[1], p[2]);
    }

    // write normals
    auto normals = mesh.get_vertex_property<pmp::Normal>("v:normal");
    const bool write_normals = normals && io_flags.use_vertex_normals;
    if (write_normals)
    {
        for (auto v : mesh.vertices())
        {
            const pmp::Normal& n = normals[v];
            fprintf(out, "vn %.7f %.7f %.7f\n", n[0], n[1], n[2]);
        }
    }

    // write texture coordinates
    auto tex_coords = mesh.get_halfedge_property<pmp::TexCoord>("h:tex");
    const bool write_texcoords = tex_coords && io_flags.use_halfedge_texcoords;

    if (write_texcoords)
    {
        if (mesh.n_halfedges() > uint_max) {
            throw pmp::InvalidInputException("Mesh too large to be written with 32-bit indices.");
        }

        for (const auto h : mesh.halfedges())
        {
            const pmp::TexCoord& pt = tex_coords[h];
            // fprintf(out, "vt %.7f %.7f\n", pt[0], pt[1]);
            fprintf(out, "vt %s %s\n", trim_float(pt[0]).c_str(), trim_float(pt[1]).c_str());
        }
    }

    // write faces
    for (auto f : mesh.faces())
    {
        fprintf(out, "f");

        auto h = mesh.halfedges(f);
        for (auto v : mesh.vertices(f))
        {
            auto idx = (uint32_t)(v.idx() + 1);
            if (write_texcoords)
            {
                if (write_normals)
                {
                    // write vertex index, texCoord index and normal index
                    fprintf(out, " %d/%d/%d", idx, (uint32_t)(*h).idx() + 1,
                            idx);
                }
                else
                {
                    // write vertex index, texCoord index
                    fprintf(out, " %d/%d/", idx, (uint32_t)(*h).idx() + 1);
                }
                ++h;
            }
            else if (write_normals)
            {
                // write vertex index and normal index
                fprintf(out, " %d//%d", idx, idx);
            }
            else
            {
                // write only vertex index
                fprintf(out, " %d", idx);
            }
        }
        fprintf(out, "\n");
    }

    fclose(out);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================
