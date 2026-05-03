#pragma once
#define ASSERT(condition) { if(!(condition)){ std::cerr << "ASSERT FAILED: " << #condition << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; } }

// TODO: Remove this file

#include "volumetric_base/utility/constants.h"
#include "volumetric_base/utility/matrix_util.hpp"
#include "volumetric_base/utility/mesh_util.h"
#include "volumetric_base/utility/precompute_solver.h"
#include "volumetric_base/utility/random_util.hpp"
#include "volumetric_base/utility/timing.hpp"

#include "volumetric_base/layered_mesh.h"
