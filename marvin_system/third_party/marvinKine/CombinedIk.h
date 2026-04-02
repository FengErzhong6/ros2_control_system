#ifndef COMBINED_IK_H
#define COMBINED_IK_H

#include "FxRobot.h"
#include "FXMatrix.h"

namespace combined_ik {

struct Geometry {
    FX_INT32L robot_type;
    FX_DOUBLE dh[8][4];
    FX_DOUBLE joint_limit_min[7];
    FX_DOUBLE joint_limit_max[7];
    FX_DOUBLE joint67_limit_pp[3];
    FX_DOUBLE joint67_limit_np[3];
    FX_DOUBLE joint67_limit_nn[3];
    FX_DOUBLE joint67_limit_pn[3];

    FX_DOUBLE base_to_shoulder;
    FX_DOUBLE upper_arm_raw;
    FX_DOUBLE forearm_raw;
    FX_DOUBLE elbow_offset;
    FX_DOUBLE wrist_to_flan;

    FX_DOUBLE shoulder_to_elbow;
    FX_DOUBLE elbow_to_wrist;
};

struct CandidateScoreParams {
    FX_DOUBLE fk_compare_tol;
    FX_DOUBLE upper_arm_dir_weight;
    FX_DOUBLE magnitude_weight;
};

struct Request {
    Matrix4 target_tcp;
    Matrix4 tool;
    Vect3 v_init;
    Vect7 ref_joint_deg;
    FX_DOUBLE epsilon_tol;
    FX_INT32L fabrik_max_iterations;
    FX_INT32L fabrik_switch_nl;
    FX_INT32L optimizer_max_evals;
    CandidateScoreParams score_params;
};

struct Result {
    FX_BOOL success;
    FX_BOOL reachable;
    FX_BOOL fabrik_converged;
    FX_BOOL used_optimizer;
    FX_BOOL optimizer_success;
    FX_INT32L candidate_count;
    FX_INT32L best_index;
    FX_INT32L switch_nl;
    FX_INT32L optimizer_status;
    FX_INT32L optimizer_eval_count;
    Vect7 best_joints_deg;
    FX_DOUBLE best_fk_residual_l1;
    FX_DOUBLE best_ref_score_l1;
    FX_DOUBLE best_upper_arm_dir_score;
    FX_DOUBLE optimizer_initial_cost;
    FX_DOUBLE optimizer_final_cost;
    CandidateScoreParams score_params;
};

FX_VOID SetDefaultCandidateScoreParams(CandidateScoreParams* params);
FX_VOID SetDefaultRequest(Request* request);
FX_BOOL LoadGeometryFromMvKDCfg(const FX_CHAR* path, Geometry* geometry);
FX_BOOL Solve(const Geometry* geometry, const Request* request, Result* result);

}  // namespace combined_ik

#endif
