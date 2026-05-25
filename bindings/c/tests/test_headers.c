/**
 * @file test_headers.c
 * @brief Compile-only smoke test for split COPP C headers.
 */

#include <stddef.h>

#include "copp/core.h"
#include "copp/path.h"
#include "copp/robot.h"
#include "copp/formulation.h"
#include "copp/interpolation.h"
#include "copp/topp2.h"
#include "copp/copp2.h"
#include "copp/topp3.h"
#include "copp/copp3.h"
#include "copp/copp.h"

int main(void)
{
    enum CoppStatus status = COPP_STATUS_OK;
    enum CoppVerbosity verbosity = COPP_VERBOSITY_SILENT;
    struct CoppMatrixViewF64 view = COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(NULL, 0, 0);
    struct CoppSliceF64 slice = {NULL, 0};
    struct CoppSliceMutF64 mut_slice = {NULL, 0};
    struct CoppVecF64 vec = {NULL, 0, 0};
    struct CoppVecUsize usize_vec = {NULL, 0, 0};
    struct CoppMatrixF64 matrix = {NULL, 0, 0, 0};
    struct CoppPathOptions path_options = {
        5,
        0.0,
        1.0,
        COPP_PATH_OUT_OF_RANGE_MODE_ERROR,
        COPP_PATH_PARAMETRIZATION_UNIFORM,
        view,
        view,
    };

    struct Topp2Problem topp2_problem = {NULL, 0, 0, 0.0, 0.0};
    struct Topp2RaOptions ra_options = {1e-8, 1e-8, 1e-8, COPP_VERBOSITY_SILENT};
    struct CoppReachSet2Result reach2_result = {{NULL, 0, 0}, {NULL, 0, 0}};

    struct CoppObjective objective = {
        COPP_OBJECTIVE_KIND_TIME,
        1.0,
        slice,
        slice,
        slice,
    };
    struct Copp2Problem copp2_problem = {NULL, 0, 0, 0.0, 0.0, &objective, 1};
    struct CoppProfile3rd profile3 = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct Topp3Problem topp3_problem = {
        NULL,
        0,
        slice,
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
    };
    struct Copp3Problem copp3_problem = {
        NULL,
        0,
        slice,
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
        &objective,
        1,
    };

    enum CoppClarabelDirectSolveMethod direct_method =
        COPP_CLARABEL_DIRECT_SOLVE_METHOD_AUTO;
    enum CoppClarabelSolverStatus solver_status = COPP_CLARABEL_SOLVER_STATUS_UNSOLVED;
    struct CoppClarabelLinearSolverInfo linsolver_info;
    struct CoppClarabelSettings clarabel_settings;
    struct CoppClarabelOptions clarabel_options;
    struct Copp2SocpResult copp2_socp_result;
    struct Copp3SocpResult copp3_socp_result;

    (void)status;
    (void)verbosity;
    (void)slice;
    (void)mut_slice;
    (void)vec;
    (void)usize_vec;
    (void)matrix;
    (void)path_options;
    (void)topp2_problem;
    (void)ra_options;
    (void)reach2_result;
    (void)copp2_problem;
    (void)profile3;
    (void)topp3_problem;
    (void)copp3_problem;
    (void)objective;
    (void)direct_method;
    (void)solver_status;
    (void)linsolver_info;
    (void)clarabel_settings;
    (void)clarabel_options;
    (void)copp2_socp_result;
    (void)copp3_socp_result;

    (void)copp_version;
    (void)copp_status_message;
    (void)copp_last_error_code;
    (void)copp_last_error_message;
    (void)copp_last_error_message_len;
    (void)copp_last_error_message_copy;
    (void)copp_clear_last_error;
    (void)copp_set_last_error_message;
    (void)copp_set_last_error_message_n;
    (void)copp_vec_f64_free;
    (void)copp_vec_usize_free;
    (void)copp_matrix_f64_free;
    (void)copp_profile_3rd_free;
    (void)copp_clarabel_default_options;
    (void)copp_clarabel_solution_to_profile_2nd;
    (void)copp_clarabel_solution_to_profile_3rd;

    (void)copp_path_default_options;
    (void)copp_path_from_waypoints;
    (void)copp_path_from_evaluator_2nd;
    (void)copp_path_from_evaluator_3rd;
    (void)copp_path_dim;
    (void)copp_path_s_range;
    (void)copp_path_evaluate_up_to_2nd;
    (void)copp_path_evaluate_up_to_3rd;
    (void)copp_path_free;

    (void)copp_robot_create;
    (void)copp_robot_set_inverse_dynamics;
    (void)copp_robot_clear_inverse_dynamics;
    (void)copp_robot_append_s;
    (void)copp_robot_amax_substitute;
    (void)copp_robot_dim;
    (void)copp_robot_len;
    (void)copp_robot_capacity;
    (void)copp_robot_is_empty;
    (void)copp_robot_idx_s_start;
    (void)copp_robot_idx_s_end;
    (void)copp_robot_constraint_rows;
    (void)copp_robot_get_s;
    (void)copp_robot_get_amax;
    (void)copp_robot_s_vec;
    (void)copp_robot_amax_vec;
    (void)copp_robot_acc_constraints_at;
    (void)copp_robot_jerk_constraints_at;
    (void)copp_robot_jerk_linear_constraints_at;
    (void)copp_robot_clear_constraints;
    (void)copp_robot_pop_front_n;
    (void)copp_robot_pop_back_n;
    (void)copp_robot_pop_front_until;
    (void)copp_robot_pop_back_until;
    (void)copp_robot_sample_path_2nd;
    (void)copp_robot_sample_path_3rd;
    (void)copp_robot_set_q_2nd;
    (void)copp_robot_set_q_3rd;
    (void)copp_add_raw_constraint_1st;
    (void)copp_add_raw_constraint_2nd;
    (void)copp_add_raw_constraint_3rd;
    (void)copp_add_axial_velocity_limits;
    (void)copp_add_axial_velocity_limits_matrix;
    (void)copp_add_axial_acceleration_limits;
    (void)copp_add_axial_acceleration_limits_matrix;
    (void)copp_add_axial_torque_limits;
    (void)copp_add_axial_torque_limits_matrix;
    (void)copp_add_axial_jerk_limits;
    (void)copp_add_axial_jerk_limits_matrix;
    (void)copp_robot_free;

    (void)copp_a_to_b_2nd;
    (void)copp_s_to_t_2nd;
    (void)copp_t_to_s_uniform_2nd;
    (void)copp_t_to_s_non_uniform_2nd;
    (void)copp_s_to_t_3rd;
    (void)copp_t_to_s_uniform_3rd;
    (void)copp_t_to_s_non_uniform_3rd;
    (void)copp_force_positive_a_3rd;

    (void)topp2_ra_default_options;
    (void)topp2_ra;
    (void)copp_reach_set2_backward;
    (void)copp_reach_set2_bidirectional;
    (void)copp_reach_set2_result_free;

    (void)copp2_socp;
    (void)copp2_socp_expert;
    (void)copp2_socp_result_free;
    (void)topp3_lp;
    (void)topp3_lp_expert;
    (void)topp3_socp;
    (void)topp3_socp_expert;
    (void)copp3_socp;
    (void)copp3_socp_expert;
    (void)copp3_socp_result_free;
    return 0;
}
