// Test purpose: ensure every public C++ header can be included together.
// This catches missing includes, circular include assumptions, and namespace
// regressions before users hit them in downstream builds.

#include <copp/clarabel.hpp>
#include <copp/core.hpp>
#include <copp/copp.hpp>
#include <copp/interpolation.hpp>
#include <copp/objective.hpp>
#include <copp/path.hpp>
#include <copp/robot.hpp>
#include <copp/solver/copp2_socp.hpp>
#include <copp/solver/copp3_socp.hpp>
#include <copp/solver/reach_set2.hpp>
#include <copp/solver/topp2_ra.hpp>
#include <copp/solver/topp3.hpp>
#include <copp/solver/topp3_lp.hpp>
#include <copp/solver/topp3_socp.hpp>

int main()
{
    return copp::version().empty() ? 1 : 0;
}
