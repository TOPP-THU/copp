#pragma once

// Umbrella header for the experimental COPP C++ facade.
//
// Include this header in application code unless a smaller dependency surface is
// needed. The implementation is intentionally not header-only; public classes in
// these headers call into `.cpp` files that hide the private `cxx` bridge.

#include "copp/core.hpp"
#include "copp/clarabel.hpp"
#include "copp/interpolation.hpp"
#include "copp/objective.hpp"
#include "copp/path.hpp"
#include "copp/robot.hpp"
#include "copp/solver/copp2_socp.hpp"
#include "copp/solver/copp3_socp.hpp"
#include "copp/solver/reach_set2.hpp"
#include "copp/solver/topp2_ra.hpp"
#include "copp/solver/topp3.hpp"
#include "copp/solver/topp3_lp.hpp"
#include "copp/solver/topp3_socp.hpp"
