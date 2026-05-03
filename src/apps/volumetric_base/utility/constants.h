#pragma once

#include <string>
#include <pmp/types.h>

#include "volumetric_base/volumetric_types.h"

NAMESPACE_BEGIN(volumetric_base)
// todo: @fabian add namespace
// NAMESPACE_BEGIN(constants)

// TODO: Better constant management

const std::string VP_POSITION = "v:point";
const std::string VP_SKELETON_POSITION = "vp: skeleton position";
const std::string VP_MUSCLE_POSITION = "vp: muscle position";
const std::string VP_SKIN_POSITION = "vp: skin position";

const std::string FP_CELL_PROLONGATED_POINT_SKELETON =
    "fp: prolongated cell point skeleton";
const std::string EP_QUAD_PROLONGATED_POINT_SKELETON =
    "ep: prolongated quad point skeleton";
const std::string FP_CELL_PROLONGATED_POINT_MUSCLE =
    "fp: prolongated cell point muscle";
const std::string EP_QUAD_PROLONGATED_POINT_MUSCLE =
    "ep: prolongated quad point muscle";
const std::string VP_OLD_INDICES = "vp: indices before cleaning";
const std::string FP_OLD_INDICES = "fp: indices before cleaning";

const std::string VP_DEPENDS = "v:depends";
const std::string VP_SKINNING_WEIGHTS = "v:weights";
constexpr double pointsize = 0.3;
inline pmp::Point quadColor(1.0f, 0.0f, 0.0f);
inline pmp::Point cellColor(0.3f, 0.6f, 0.8f);
inline std::vector<Layer> allMeshTypes = {Layer::SKEL, Layer::SKIN, Layer::MUSC};


inline std::tuple<double, double> lame_from_params(std::tuple<double,double> in) {
    auto [nu, E] = in;
    return {(E*nu)/((1+nu)*(1-2*nu)),E/(2+2*nu)};
}

const std::tuple<double,double> muscle_params = {0.47, 15};
const std::tuple<double,double> skin_params = {0.47, 5};
const std::tuple<double,double> muscle_lame = lame_from_params(muscle_params);
const std::tuple<double,double> skin_lame = lame_from_params(skin_params);

// NAMESPACE_END(constants)
NAMESPACE_END(volumetric_base)
