#ifndef TRACKING_IK_H
#define TRACKING_IK_H

#include "FxRobot.h"
#include "FXMatrix.h"

namespace tracking_ik {

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
    FX_DOUBLE desired_dir_weight;
    FX_DOUBLE continuity_dir_weight;
    FX_DOUBLE magnitude_weight;
    FX_DOUBLE psi_delta_weight;
    FX_DOUBLE branch_switch_penalty;
};

struct Request {
    Matrix4 target_tcp;
    Matrix4 tool;
    Vect3 desired_upper_arm_dir;
    Vect3 prev_selected_ref_dir;
    Vect7 ref_joint_deg;
    FX_INT32L prev_selected_branch;
    FX_DOUBLE fk_accept_tol;
    FX_DOUBLE fine_psi_range_deg;
    FX_DOUBLE fine_psi_step_deg;
    FX_DOUBLE fast_psi_range_deg;
    FX_DOUBLE fast_psi_step_deg;
    FX_DOUBLE expand_psi_range_deg;
    FX_DOUBLE expand_psi_step_deg;
    CandidateScoreParams score_params;
};

struct Result {
    FX_BOOL success;
    FX_BOOL reachable;
    FX_BOOL used_expanded_search;
    FX_INT32L psi_eval_count;
    FX_INT32L candidate_count;
    FX_INT32L best_index;
    FX_INT32L selected_branch;
    FX_DOUBLE selected_psi_deg;
    Vect3 selected_ref_dir;
    Vect7 best_joints_deg;
    FX_DOUBLE best_fk_residual_l1;
    FX_DOUBLE best_ref_score_l1;
    FX_DOUBLE best_desired_dir_score;
    FX_DOUBLE best_continuity_dir_score;
    CandidateScoreParams score_params;
};

FX_VOID SetDefaultCandidateScoreParams(CandidateScoreParams* params);
FX_VOID SetDefaultRequest(Request* request);
FX_BOOL LoadGeometryFromMvKDCfg(const FX_CHAR* path, Geometry* geometry);
FX_BOOL Solve(const Geometry* geometry, const Request* request, Result* result);

}  // namespace tracking_ik

#endif
