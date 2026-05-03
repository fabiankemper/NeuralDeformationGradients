//
// Created by peasman on 3/12/25.
//
#include "fbx_export.h"

#include <iostream>

#include <pmp/io/io.h>
#include "pmp_character/stb_image/stb_image_write.h"

using namespace pmp;
using namespace pmp_character;
using namespace fbxsdk;

namespace {
FbxManager* mFbxManager;
FbxScene* mFbxScene;
FbxNode* root;

// Utility function to convert a 4x4 joint transform matrix into an FBX matrix
// and decompose it into translation/rotation/scale.
static void DecomposeJointTransform(const mat4& jointTransform, FbxDouble3& outTranslation, FbxDouble3& outRotation,
                                    FbxDouble3& outScaling)
{
    // Create an FBX matrix from the jointTransform.
    FbxAMatrix fbxMat;
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            fbxMat[r][c] = jointTransform(c, r);
        }
    }

    // Decompose into T, R, S
    FbxVector4 t = fbxMat.GetT();
    FbxVector4 r = fbxMat.GetR();
    FbxVector4 s = fbxMat.GetS();

    outTranslation = FbxDouble3(t[0], t[1], t[2]);
    outRotation = FbxDouble3(r[0], r[1], r[2]);
    outScaling = FbxDouble3(s[0], s[1], s[2]);
}
} // namespace

namespace FBXExporter {
void CreateBindPose(Skeleton& skeleton)
{
    FbxPose* bindPose = FbxPose::Create(mFbxScene, "BindPose");
    bindPose->SetIsBindPose(true);

    for (auto& joint : skeleton.joints_)
    {
        FbxNode* jointNode = mFbxScene->FindNodeByName(joint->name_.c_str());
        if (!jointNode)
            continue;

        // Convert joint.local_initial (mat4) to FbxAMatrix
        FbxAMatrix bindMatrix;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                bindMatrix[r][c] = joint->global_initial_(c, r);

        bindPose->Add(jointNode, bindMatrix);
    }

    // Add bind pose to the scene
    mFbxScene->AddPose(bindPose);
}

bool ExportCharacter(Character* character, const std::string& outputFile)
{
    bones.clear();
    if (!InitializeFbxSdk())
    {
        std::cerr << "Failed to initialize FBX SDK." << std::endl;
        return false;
    }

    CreateSkeletonNodes(character);
    CreateBindPose(character->skeleton_);
    CreateMeshNodes(character);
    CreateBlendShapeNodes(character);
    SetScanPose(character);

    bool result = SaveScene(outputFile);

    if (mFbxManager)
    {
        mFbxManager->Destroy();
        mFbxManager = nullptr;
    }

    return result;
}

bool InitializeFbxSdk()
{
    mFbxManager = FbxManager::Create();
    if (!mFbxManager)
        return false;

    FbxIOSettings* ios = FbxIOSettings::Create(mFbxManager, IOSROOT);
    mFbxManager->SetIOSettings(ios);
    mFbxScene = FbxScene::Create(mFbxManager, "CharacterScene");
    mFbxScene->GetGlobalSettings().SetSystemUnit(FbxSystemUnit::m);
    root = FbxNode::Create(mFbxScene, "Character");
    root->Visibility = 0.0;
    mFbxScene->GetRootNode()->AddChild(root);
    return mFbxScene != nullptr;
}

void CreateSkeletonNodes(Character* character)
{
    Skeleton& skeleton = character->skeleton_;

    // Resize bone vector to the number of joints
    bones.resize(skeleton.joints_.size());

    Joint* rootJoint = skeleton.root_;
    if (rootJoint)
    {
        AddJointHierarchy(rootJoint, root);
    }
}

void AddJointHierarchy(Joint* joint, FbxNode* parentNode)
{
    FbxSkeleton* fbxSkeletonAttr = FbxSkeleton::Create(mFbxScene, joint->name_.c_str());
    fbxSkeletonAttr->SetSkeletonType(joint->is_root() ? FbxSkeleton::eRoot : FbxSkeleton::eLimbNode);

    FbxNode* jointNode = FbxNode::Create(mFbxScene, joint->name_.c_str());
    jointNode->SetNodeAttribute(fbxSkeletonAttr);

    FbxDouble3 translation, rotation, scaling;
    DecomposeJointTransform(joint->local_initial_, translation, rotation, scaling);
    jointNode->LclTranslation.Set(translation);
    jointNode->LclRotation.Set(rotation);
    jointNode->LclScaling.Set(scaling);

    // Properly store the joint node reference
    bones[joint->id] = jointNode;

    parentNode->AddChild(jointNode);

    for (Joint* child : joint->children_)
    {
        AddJointHierarchy(child, jointNode);
    }
}
void SetScanPose(Character* character)
{
    for (size_t i = 0; i < character->skeleton_.joints_.size(); i++)
    {
        auto& joint = character->skeleton_.joints_[i];
        auto& fbxJoint = bones[i];
        FbxDouble3 translation, rotation, scaling;
        DecomposeJointTransform(joint->local_, translation, rotation, scaling);
        fbxJoint->LclTranslation.Set(translation);
        fbxJoint->LclRotation.Set(rotation);
        fbxJoint->LclScaling.Set(scaling);
    }
}

//-----------------------------------------------------------------------------
// CreateMeshNodes now iterates over child SceneNodes of the character.
// Each child that can be cast to SkinnedMeshNode is exported as a submesh.
//-----------------------------------------------------------------------------

void CreateMeshNodes(Character* character)
{
    // Retrieve the character's children (SceneNodes).
    const auto& children = character->getChildren();

    int submeshIndex = 0;
    for (auto& child : children)
    {
        // Attempt to cast to SkinnedMeshNode.
        std::shared_ptr<MeshNode> skinned = std::dynamic_pointer_cast<MeshNode>(child);
        if (!skinned)
            continue; // not a mesh node.

        // Create a dedicated FBX node for this submesh.
        std::string nodeName = skinned->name;
        FbxNode* meshNode = FbxNode::Create(mFbxScene, nodeName.c_str());

        // Retrieve the underlying PMP mesh.
        auto& mesh = skinned->mesh;

        // Create an FBX mesh.
        FbxMesh* fbxMesh = FbxMesh::Create(mFbxScene, skinned->name.c_str());
        fbxMesh->InitControlPoints(static_cast<int>(mesh.n_vertices()));
        // Fill control points.
        {
            size_t index = 0;
            for (auto v_it : mesh.vertices())
            {
                auto point = mesh.position(v_it);
                fbxMesh->GetControlPoints()[index++] = FbxVector4(point[0], point[1], point[2]);
            }
        }

        const auto vtx_bone_indices = mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
        const auto vtx_bone_weights = mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);

        FbxSkin* fbxSkin = FbxSkin::Create(mFbxScene, "MeshSkinDeformer");
        FbxAMatrix global_mesh_transform = meshNode->EvaluateGlobalTransform();

        std::map<size_t, FbxCluster*> used_clusters;

        for (auto v_it : mesh.vertices())
        {
            const auto& boneIndices = vtx_bone_indices[v_it];
            const auto& boneWeights = vtx_bone_weights[v_it];

            for (int i = 0; i < 8; ++i)
            {
                if (boneWeights[i] > 0.0000001f)
                {
                    size_t boneIndex = static_cast<size_t>(boneIndices[i]);

                    FbxCluster*& cluster = used_clusters[boneIndex];
                    if (!cluster)
                    {
                        // Create cluster and set the associated bone
                        cluster = FbxCluster::Create(mFbxScene, bones[boneIndex]->GetName());
                        cluster->SetLink(bones[boneIndex]);
                        cluster->SetLinkMode(FbxCluster::eNormalize);
                        cluster->SetTransformMatrix(global_mesh_transform);
                        cluster->SetTransformLinkMatrix(bones[boneIndex]->EvaluateGlobalTransform());
                        fbxSkin->AddCluster(cluster);
                    }

                    cluster->AddControlPointIndex(v_it.idx(), static_cast<double>(boneWeights[i]));
                }
            }
        }

        fbxMesh->AddDeformer(fbxSkin);

        // Fill polygon data.
        for (auto f_it : mesh.faces())
        {
            fbxMesh->BeginPolygon();
            for (auto fv_it : mesh.vertices(f_it))
            {
                fbxMesh->AddPolygon(fv_it.idx());
            }
            fbxMesh->EndPolygon();
        }

        // Optionally handle UV data if available in SkinnedMeshNode or 'mesh'.
        // For example:
        if (mesh.has_halfedge_property(properties::halfedgeTextureCoordinates))
        {
            // Retrieve halfedge texture coordinates from PMP mesh
            auto he_tex = mesh.halfedge_property<pmp::TexCoord>(properties::halfedgeTextureCoordinates);

            // Create the UV element for the FBX mesh
            FbxGeometryElementUV* uvElement = fbxMesh->CreateElementUV("UVSet0");
            uvElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
            uvElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

            // Temporary container for indexing UV coordinates
            std::vector<FbxVector2> uvCoords;
            std::map<pmp::Halfedge, int> uvIndices;

            // Iterate over faces and vertices to populate UV data
            for (auto f_it : mesh.faces())
            {
                for (auto fv_it : mesh.halfedges(f_it))
                {
                    auto uv = he_tex[fv_it];
                    FbxVector2 fbxUV(uv[0], 1.0 - uv[1]);

                    auto inserted = uvIndices.insert({fv_it, static_cast<int>(uvCoords.size())});
                    if (inserted.second) // was newly inserted
                    {
                        uvCoords.push_back(fbxUV);
                    }

                    // Add index to the FBX UV element
                    uvElement->GetIndexArray().Add(inserted.first->second);
                }
            }

            // Add UV coordinates to FBX DirectArray
            for (const auto& uvCoord : uvCoords)
            {
                uvElement->GetDirectArray().Add(uvCoord);
            }
        }

        // Attach the mesh to the node.
        meshNode->SetNodeAttribute(fbxMesh);
        root->AddChild(meshNode);

        //---------------------------------------------------------------------------
        // Create a basic material/texture if available in the SkinnedMeshNode.
        // Suppose the node has something like:
        //   skinned->getTexturePath() => returns a string path
        //   skinned->getDiffuseColor() => returns a vec3 or array
        // Modify to match your actual code.
        //---------------------------------------------------------------------------
        {
            // Example usage:
            // if (!skinned->getTexturePath().empty() || skinned->hasMaterialInfo()) {
            // Create a basic Phong material.
            FbxSurfacePhong* material =
                FbxSurfacePhong::Create(mFbxScene, ("Material_" + std::to_string(submeshIndex)).c_str());
            material->ShadingModel.Set("Phong");
            // Diffuse color factor.
            material->DiffuseFactor.Set(1.0);
            material->ReflectionFactor.Set(1.0);
            double roughness = 1.0;                             // Roughness between 0 (smooth) and 1 (rough)
            double shininessExponent = (1.0 - roughness) * 128; // invert roughness
            material->Shininess.Set(shininessExponent);
            // In case we have a base color in the node.

            // If there's a texture path:
            std::string texturePath = skinned->texture_name;
            if (!texturePath.empty() or character->textureAtlas.contains(texturePath))
            {
                auto& texture = character->textureAtlas[skinned->texture_name];
                if (texturePath.empty())
                {
                    texturePath = texture.filepath.string();
                }
                std::filesystem::path texture_path = texturePath;
                if (texture_path.extension().empty())
                {
                    texture_path.replace_extension(".png");
                }

                // todo: fake: Update PMP Character
                // stbi_write_png(texture_path.c_str(), texture.width, texture.height, texture.numComponents,
                //                texture.textureBuffer.data(), 0);
                FbxFileTexture* diffuseTex = FbxFileTexture::Create(mFbxScene, texture_path.string().c_str());
                diffuseTex->SetFileName(texture_path.string().c_str());
                diffuseTex->SetTextureUse(FbxTexture::eStandard);
                diffuseTex->SetMappingType(FbxTexture::eUV);
                diffuseTex->SetMaterialUse(FbxFileTexture::eModelMaterial);
                diffuseTex->SetSwapUV(false);
                diffuseTex->SetTranslation(0.0, 0.0);
                diffuseTex->SetScale(1.0, 1.0);
                diffuseTex->SetRotation(0.0, 0.0);

                material->Diffuse.ConnectSrcObject(diffuseTex);
            }

            // Add the material to the mesh node.
            meshNode->AddMaterial(material);
            // }
        }
        //---------------------------------------------------------------------------

        submeshIndex++;
    }
}

//-----------------------------------------------------------------------------
// CreateBlendShapeNodes iterates over the same child SkinnedMeshNodes.
// Each submesh's SkinnedMeshNode might have its own blendshapes.
//-----------------------------------------------------------------------------
void CreateBlendShapeNodes(Character* character)
{
    // For each SkinnedMeshNode, find its corresponding FBX node and create blendshapes.
    const auto& children = character->getChildren();

    for (auto& child : children)
    {
        std::shared_ptr<MeshNode> skinned = std::dynamic_pointer_cast<MeshNode>(child);
        if (!skinned)
            continue;

        // If no blendshapes exist in this submesh, skip.
        if (skinned->blendshape_collection.empty)
        {
            std::cout << "[DEBUG] No Blendshapes found for " << skinned->name << std::endl;
            continue;
        }
        // Construct the name used to find the FBX node.
        std::string nodeName = skinned->name;
        FbxNode* meshNode = mFbxScene->GetRootNode()->FindChild(nodeName.c_str());
        if (!meshNode)
        {
            // If we didn't find a node, perhaps it's under "MeshesRoot".
            // We can more precisely search by exact name in the root's children, or store a pointer earlier.
            // For simplicity, do a direct find:
            meshNode = mFbxScene->FindNodeByName(nodeName.c_str());
            if (!meshNode)
            {
                continue;
            }
        }

        FbxMesh* fbxMesh = meshNode->GetMesh();
        if (!fbxMesh)
        {
            continue;
        }

        // // TODO: Add Blendshape support
        // // Create a blendshape deformer.
        // FbxBlendShape* blendShapeDeformer =
        //     FbxBlendShape::Create(mFbxScene, ("BlendShapeDeformer_" + nodeName).c_str());
        //
        // // Retrieve submesh's blendshapes.
        // auto& blendshapes = skinned->blendshape_collection.blendshapes;
        // for (auto& bs : blendshapes)
        // {
        //     // Create a new channel for each blendshape.
        //     FbxBlendShapeChannel* channel = FbxBlendShapeChannel::Create(mFbxScene, bs.name.c_str());
        //     FbxShape* targetShape = FbxShape::Create(mFbxScene, bs.name.c_str());
        //
        //     // Populate the shape's control points.
        //     auto bsMesh = bs.getBlendshapeMesh();
        //     if (child->name == "skin")
        //         pmp::write(bsMesh, bs.name + ".obj");
        //     targetShape->InitControlPoints(static_cast<int>(bsMesh.n_vertices()));
        //     std::string name = channel->GetName();
        //
        //     size_t index = 0;
        //     for (auto v_it : bsMesh.vertices())
        //     {
        //         auto point = bsMesh.position(v_it);
        //         targetShape->GetControlPoints()[index++] = FbxVector4(point[0], point[1], point[2]);
        //     }
        //
        //     channel->AddTargetShape(targetShape);
        //     blendShapeDeformer->AddBlendShapeChannel(channel);
        // }
        //
        // // Attach the deformer to the FBX mesh.
        // fbxMesh->AddDeformer(blendShapeDeformer);
    }
}

bool SaveScene(const std::string& outputFile)
{
    FbxExporter* exporter = FbxExporter::Create(mFbxManager, "");
    auto ios = mFbxManager->GetIOSettings();
    ios->SetBoolProp(EXP_FBX_TEXTURE, false);
    if (!exporter->Initialize(outputFile.c_str(), -1, ios))
    {
        exporter->Destroy();
        return false;
    }

    bool result = exporter->Export(mFbxScene);
    exporter->Destroy();
    return result;
}
} // namespace FBXExporter
