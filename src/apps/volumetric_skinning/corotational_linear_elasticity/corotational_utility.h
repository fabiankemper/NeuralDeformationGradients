//
// Created by gerrit on 09.07.24.
//

#pragma once
#ifndef VIRTUAL_SKINNING_H
#define VIRTUAL_SKINNING_H

#include "volumetric_base/utility/utility.h"
#include "volumetric_character/volumetric_character.h"


class CorotationUtility {
public:

    void ComputeRotationsPerVertex(const volumetric_base::LayeredMesh &mesh, const volumetric_character::VolumetricCharacter &character);

    CorotationUtility(volumetric_base::LayeredMesh &mesh, volumetric_character::VolumetricCharacter &character, pmp::SurfaceMesh &propSource);
    [[nodiscard]] std::tuple<Eigen::Quaterniond,Eigen::Vector3d> get_rotation_and_translation(int i1,int i2,int i3,int i4) const;
    std::tuple<Eigen::Matrix3d,Eigen::Vector3d> get_rotation_and_translation(const std::vector<int>& indices);

    [[nodiscard]] std::vector<std::map<int,double>> get_skinning_weights() const;
    std::vector<std::map<int,double>> skinning_weights;
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d>  translations;

};



#endif //VIRTUAL_SKINNING_H
