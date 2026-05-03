//
// Created by peasman on 10.01.24.
//

#ifndef MESHUTILITIES_H
#define MESHUTILITIES_H

#include <pmp/surface_mesh.h>

#include "../node_types/Character.h"
#include "../Types.h"

namespace pmp_character::mesh_utilities
{
    static const pmp::TexCoord DEFAULT_TEXTURE_COORDINATE(-1,-1);
    static const float EPSILON = 0.0001f;


    [[maybe_unused]] static void detectDuplicateVertices(pmp::SurfaceMesh& mesh, std::vector<pmp::Point>& uniquePositions, std::vector<pmp::Vertex>& uniqueIndices)
    {
        auto originalIndices = mesh.add_vertex_property<int>("v:originalIndex");
        // Care for indices
        for (pmp::Vertex v: mesh.vertices())
        {
            auto find = std::find(uniquePositions.begin(), uniquePositions.end(), mesh.position(v));
            int index = find - uniquePositions.begin();
            if(find == uniquePositions.end())
            {
                uniquePositions.emplace_back(mesh.position(v));
                uniqueIndices.emplace_back(v.idx());
                originalIndices[v] = index;
            } else
            {
                // printf("Non unique Vertex found at: %d \n",index);
                // Get the index in the list of original vertices
                originalIndices[v] = index;
                // std::cout << "OG Vertex: " << originalIndices[v] << std::endl;
            }
        }
    }


    /// \brief Function for deleting all duplicate Vertices and closing seems in the mesh
    /// \param baseMesh
    /// \param targetMesh


    /// \brief Creates new deformations for vertices that got duplicated
    /// \param character The character, where the blendshape is stored
    /// \param childIndex The index of the child, where the mesh is stored
    [[maybe_unused]] static void duplicateBlendshapeEntries(Character &character, int childIndex)
    {
        pmp::SurfaceMesh &mesh = character.get_mesh_by_index(childIndex)->mesh;
        // Gather all duplicated vertices in lists of lists
        if (not mesh.has_vertex_property(properties::originalIndex))
        {
            return;
        }
        // Gather all sets of duplicated vertices for a mesh
        auto originalIndices = mesh.get_vertex_property<int>(properties::originalIndex);
        std::vector<std::vector<pmp::Vertex>> duplicateVertices;
        for (pmp::Vertex v : mesh.vertices())
        {
            if (originalIndices[v] != (int)v.idx())
            {
                duplicateVertices[originalIndices[v]].emplace_back(v);
            }
            else
            {
                std::vector<pmp::Vertex> duplicates;
                duplicates.emplace_back(v);
                duplicateVertices.emplace_back(duplicates);
            }
        }
        // Duplicate all moving vertices that have a duplicate in the mesh
        for (Blendshape &blendshape : character.get_mesh_by_index(childIndex)->blendshape_collection.blendshapes)
        {
            for (std::vector<pmp::Vertex> duplicateVertex : duplicateVertices)
            {
                if (duplicateVertex.size() > 1)
                {
                    // Get the first one as reference to the deformation
                    pmp::Vertex first = duplicateVertex.front();
                    // Find the index in the list of movingVertices
                    auto entry = std::find(blendshape.movingVertices.begin(), blendshape.movingVertices.end(), first);
                    // Get the corresponding Deformation
                    if(entry != blendshape.movingVertices.end())
                    {
                        pmp::Point deformation = blendshape.deformation[entry - blendshape.movingVertices.begin()];
                        // Ignore the first one
                        for (size_t i = 1; i < duplicateVertex.size(); i++)
                        {
                            pmp::Vertex duplicate = duplicateVertex[i];
                            blendshape.movingVertices.emplace_back(duplicate);
                            blendshape.deformation.emplace_back(deformation);
                        }
                    }
                    auto normalEntry = std::find(blendshape.movingNormals.begin(), blendshape.movingNormals.end(), first);
                    if(normalEntry != blendshape.movingNormals.end())
                    {
                        pmp::Point normalDeformation = blendshape.normalDeformation[normalEntry - blendshape.movingNormals.begin()];
                        // Ignore the first one
                        for (size_t i = 1; i < duplicateVertex.size(); i++)
                        {
                            pmp::Vertex duplicate = duplicateVertex[i];
                            blendshape.movingNormals.emplace_back(duplicate);
                            blendshape.normalDeformation.emplace_back(normalDeformation);
                        }
                    }

                }
            }
        }
    }
    /// \brief Helperfunction that fills a list with the vertices of a face
    /// \param mesh
    /// \param face
    /// \param vertices
    [[maybe_unused]]static void getFaceVertices(pmp::SurfaceMesh &mesh, pmp::Face face, std::vector<pmp::Vertex> &vertices)
    {
        for (pmp::Vertex v : mesh.vertices(face))
        {
            vertices.emplace_back(v);
        }
    }
    /// \brief Gets all Vertices from a halfedge and creates a face out of it
    /// \param mesh
    /// \param he
    /// \param targetMesh
    [[maybe_unused]] static void createFaceFromHe(pmp::SurfaceMesh &mesh, pmp::Halfedge he, pmp::SurfaceMesh &targetMesh)
    {
        pmp::Face face = mesh.face(he);
        std::vector<pmp::Vertex> vertices;
        getFaceVertices(mesh, face, vertices);
        targetMesh.add_face(vertices);
    }

    /// \brief Fills a list of TexCoords with the TexCoords provided by the Halfedges of the Face
    /// \param mesh The mesh where the coordinates are stored
    /// \param face The face for which the coordinates are defined
    /// \param faceTextureCoordinates out parameter for the list of TexCoords
    [[maybe_unused]] static void getFaceHalfEdgeTextureCoordinates(pmp::SurfaceMesh &mesh, pmp::Face face,
                                                  std::vector<pmp::TexCoord> &faceTextureCoordinates)
    {
        pmp::HalfedgeProperty<pmp::TexCoord> texCoords =
            mesh.get_halfedge_property<pmp::TexCoord>(properties::halfedgeTextureCoordinates);
        for (pmp::Halfedge he : mesh.halfedges(face))
        {
            faceTextureCoordinates.emplace_back(texCoords[he]);
        }
    }
    /// \brief Fills a list of TexCoords with the VertexTextureCoordingates of the given face
    /// \param mesh
    /// \param vertices
    /// \param faceTextureCoordinates
   [[maybe_unused]] static void getFaceVertexTextureCoordinates(pmp::SurfaceMesh &mesh, std::vector<pmp::Vertex> &vertices,
                                                std::vector<pmp::TexCoord> &faceTextureCoordinates)
    {
        pmp::VertexProperty<pmp::TexCoord> texCoords = mesh.get_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates);
        for (pmp::Vertex v : vertices)
        {
            faceTextureCoordinates.emplace_back(texCoords[v]);
        }
    }
    /// \brief Duplicates all vertices that got multiple TexCoords defined by HalfEdges and modifies Faces accordingly
    /// \param child The MeshNode where duplicates are needed to be duplicated
    [[maybe_unused]] static void duplicateVerticesForExport(MeshNode &child, pmp::SurfaceMesh& meshCopy)
    {
        pmp::SurfaceMesh &mesh = child.mesh;
        // Grab all old Properties
        pmp::VertexProperty<pmp::Normal> oldvNormals = mesh.get_vertex_property<pmp::Normal>(properties::vertexNormals);
        pmp::VertexProperty<ivec8> oldDepends = mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
        pmp::VertexProperty<vec8> oldWeights = mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);

        // Define Properties for MeshCopy
        pmp::VertexProperty<pmp::Normal> vNormals = meshCopy.add_vertex_property<pmp::Normal>(properties::vertexNormals);
        pmp::VertexProperty<pmp::TexCoord> vTexCoords =
            meshCopy.add_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates, DEFAULT_TEXTURE_COORDINATE);
        pmp::VertexProperty<ivec8> vDepends = meshCopy.add_vertex_property<ivec8>(properties::vertexSkinningDepends);
        pmp::VertexProperty<vec8> vWeights = meshCopy.add_vertex_property<vec8>(properties::vertexSkinningWeights);
        pmp::VertexProperty<int> originalIndices = meshCopy.add_vertex_property<int>(properties::originalIndex);
        // Store a mapping of duplicated vertex + texture coordinate
        std::vector<std::vector<pmp::Vertex>> duplicateVertexSets(mesh.n_vertices());
        std::vector<std::vector<pmp::TexCoord>> duplicateTexCoordsSets(mesh.n_vertices());
        for (pmp::Vertex v : mesh.vertices())
        {
            meshCopy.add_vertex(mesh.position(v));
        }
        for (pmp::Face f : mesh.faces())
        {
            std::vector<pmp::Vertex> faceVertices;
            std::vector<pmp::TexCoord> faceHeTexCoords;
            std::vector<pmp::TexCoord> faceVertexTexCoords;
            getFaceVertices(mesh, f, faceVertices);
            getFaceHalfEdgeTextureCoordinates(mesh, f, faceHeTexCoords);
            getFaceVertexTextureCoordinates(meshCopy, faceVertices, faceVertexTexCoords);
            for (size_t i = 0; i < faceHeTexCoords.size(); i++)
            {
                // Check if TexCoords match, so it is the non duplicated vertex
                if (norm((faceHeTexCoords[i] - faceVertexTexCoords[i])) >= EPSILON)
                {
                    // It is a duplicated Vertex
                    // Is it already been duplicated?
                    // Get the current duplicateVertex set
                    auto& duplicateSet = duplicateVertexSets[originalIndices[faceVertices[i]]];
                    auto& duplicateTextureSet = duplicateTexCoordsSets[originalIndices[faceVertices[i]]];
                    auto texturesetFind = std::find(duplicateTextureSet.begin(), duplicateTextureSet.end(), faceHeTexCoords[i]);
                    if(texturesetFind != duplicateTextureSet.end())
                    {
                        ptrdiff_t index = texturesetFind - duplicateTextureSet.begin();
                        // Get the Vertex that was duplicated
                        pmp::Vertex dup = duplicateSet[index];
                        faceVertices[i] = dup;
                    } else
                    // If it is -1 -1, simply overwrite it
                    if (norm(faceVertexTexCoords[i] - DEFAULT_TEXTURE_COORDINATE) < EPSILON)
                    {
                        originalIndices[faceVertices[i]] = faceVertices[i].idx();
                        vTexCoords[faceVertices[i]] = faceHeTexCoords[i];
                        duplicateVertexSets[faceVertices[i].idx()].emplace_back(faceVertices[i]);
                        duplicateTexCoordsSets[faceVertices[i].idx()].emplace_back(faceHeTexCoords[i]);
                        vWeights[faceVertices[i]] = oldWeights[faceVertices[i]];
                        vDepends[faceVertices[i]] = oldDepends[faceVertices[i]];
                        vNormals[faceVertices[i]] = oldvNormals[faceVertices[i]];
                    }
                    // If it is the same we dont need to do anything
                    // If it is different we need to create a new Vertex
                    else
                    {
                        auto pos = mesh.position(faceVertices[i]);
                        auto newVertex = meshCopy.add_vertex(pos);
                        vTexCoords[newVertex] = faceHeTexCoords[i];
                        originalIndices[newVertex] = faceVertices[i].idx();
                        duplicateVertexSets[faceVertices[i].idx()].emplace_back(newVertex);
                        duplicateTexCoordsSets[faceVertices[i].idx()].emplace_back(faceHeTexCoords[i]);
                        vWeights[newVertex] = oldWeights[faceVertices[i]];
                        vDepends[newVertex] = oldDepends[faceVertices[i]];
                        vNormals[newVertex] = oldvNormals[faceVertices[i]];
                        faceVertices[i] = newVertex;
                    }
                    //
                }
            }
            try
            {
                meshCopy.add_face(faceVertices);
            }  catch (pmp::TopologyException &e)
            {
                std::cerr << "Topology exception with vertices " << faceVertices[0] << ",\t" << faceVertices[1] << ",\t"
                        << faceVertices[2] << " on Mesh with " << child.mesh.n_vertices() << " vertices on face " << f <<  std::endl;
            }
        }
        printf("Changed number of Vertices from %zu to %zu \n", child.mesh.n_vertices(), meshCopy.n_vertices());
        // child.mesh = meshCopy;
    }


}

#endif //MESHUTILITIES_H
