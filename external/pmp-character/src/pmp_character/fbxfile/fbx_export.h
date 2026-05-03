//
// Created by peasman on 3/12/25.
//

#ifndef FBX_EXPORT_HPP
#define FBX_EXPORT_HPP
#include "../Joint.h"
#include "../node_types/Character.h"
#include "fbx_warnings.h"
FBX_DISABLE_WARNINGS_BEGIN
#include <fbxsdk.h>
FBX_DISABLE_WARNINGS_END
#include <pmp/mat_vec.h>

namespace FBXExporter {
static std::vector<FbxNode*> bones;

bool InitializeFbxSdk();
void CreateSkeletonNodes(pmp_character::Character* character);
bool ExportCharacter(pmp_character::Character* character, const std::string& outputFile);
void AddJointHierarchy(pmp_character::Joint* joint, FbxNode* parentNode);
void CreateMeshNodes(pmp_character::Character* character);
void CreateBlendShapeNodes(pmp_character::Character* character);
bool SaveScene(const std::string& outputFile);
void SetScanPose(pmp_character::Character* character);

} // namespace FBXExporter

#endif // FBX_EXPORT_HPP
