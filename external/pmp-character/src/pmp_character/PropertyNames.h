//
// Created by peasman on 04.01.24.
//

#ifndef PROPERTYNAMES_H
#define PROPERTYNAMES_H
#include <string>

namespace pmp_character::properties
{
	const static std::string vertexPosition = "v:point";
	const static std::string vertexSkinningWeights = "v:weights";
	const static std::string vertexSkinningDepends = "v:depends";
	const static std::string vertexNormals = "v:normal";
	const static std::string vertexTextureCoordinates = "v:tex";
	const static std::string vertexSkinnedPos = "v:skinned_pos";
	const static std::string vertexSkinnedNormal = "v:skinned_normal";
	const static std::string vertexBlendPosition = "v:blendpos";
	const static std::string vertexBlendNormal = "v:blendnormal";
	const static std::string vertexColor = "v:color";
	const static std::string vertexGlobalPosition = "v:global";
	const static std::string vertexGlobalNormal = "v:globalnormal";
	const static std::string halfedgeNormal = "h:normal";
	const static std::string halfedgeTextureCoordinates = "h:tex";
	const static std::string faceColor = "f:color";
	const static std::string originalIndex = "v:original_index";
}

#endif //PROPERTYNAMES_H
