#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "FxRobot.h"

namespace {

constexpr size_t kJoints = 7;
constexpr size_t kMaxCandidates = 8;
constexpr double kDeg2Rad = 0.01745329251994329576923690768489;
constexpr double kRad2Deg = 57.295779513082320876798154814105;
constexpr double kMm2M = 0.001;
constexpr double kM2Mm = 1000.0;

struct MarvinKineData {
  FX_INT32L type[2]{};
  FX_DOUBLE grv[2][3]{};
  FX_DOUBLE dh[2][8][4]{};
  FX_DOUBLE pnva[2][7][4]{};
  FX_DOUBLE bd[2][4][3]{};
  FX_DOUBLE mass[2][7]{};
  FX_DOUBLE mcp[2][7][3]{};
  FX_DOUBLE inertia[2][7][6]{};
};

struct Options {
  std::string kine_config_path;
  int arm = 0;
  std::array<double, kJoints> ref_joint_deg{};
  std::array<double, 3> pos_m{};
  std::array<double, 4> quat_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> elbow_dir{{0.0, 0.0, -1.0}};
  double zsp_angle_deg = 0.0;
  double dh_d1_m = 0.0;
  bool input_pos_is_shoulder_frame = false;
  bool run_near_dir = true;
  bool run_near_ref = true;
};

struct LimitCheck {
  bool in_limits{true};
  std::array<double, kJoints> run_limit_pos{};
  std::array<double, kJoints> run_limit_neg{};
  std::array<bool, kJoints> exceeded{};
  double max_exceed_abs{0.0};
};

std::string trim(const std::string &value)
{
  const auto begin = value.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t");
  return value.substr(begin, end - begin + 1);
}

std::vector<double> parseList(const std::string &raw, size_t expected, const std::string &name)
{
  std::string normalized = raw;
  for (char &ch : normalized) {
    if (ch == ',' || ch == ';' || ch == '[' || ch == ']') {
      ch = ' ';
    }
  }

  std::stringstream ss(normalized);
  std::vector<double> values;
  double value = 0.0;
  while (ss >> value) {
    values.push_back(value);
  }

  if (values.size() != expected) {
    std::ostringstream err;
    err << "Expected " << expected << " values for " << name
        << ", got " << values.size() << ".";
    throw std::runtime_error(err.str());
  }
  return values;
}

template <size_t N>
std::array<double, N> parseArray(const std::string &raw, const std::string &name)
{
  const auto values = parseList(raw, N, name);
  std::array<double, N> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

int parseArm(const std::string &raw)
{
  const std::string value = trim(raw);
  if (value == "0" || value == "left" || value == "LEFT" || value == "l") {
    return 0;
  }
  if (value == "1" || value == "right" || value == "RIGHT" || value == "r") {
    return 1;
  }
  throw std::runtime_error("Invalid --arm value, use 0/1/left/right.");
}

void printUsage(const char *argv0)
{
  std::cout
      << "Usage:\n"
      << "  " << argv0
      << " --ref \"j1,j2,j3,j4,j5,j6,j7\""
      << " --pos \"x,y,z\" --quat \"x,y,z,w\" --elbow \"x,y,z\" [options]\n\n"
      << "Options:\n"
      << "  --arm left|right|0|1          default: left\n"
      << "  --mode near_dir|near_ref|both default: both\n"
      << "  --zsp-angle deg               default: 0\n"
      << "  --dh-d1 meters                default: 0\n"
      << "  --input-frame base|shoulder   default: base\n"
      << "  --kine-config path            default: package source config\n"
      << "  --help\n\n"
      << "Example:\n"
      << "  " << argv0
      << " --arm left"
      << " --ref \"86.5,-87.4,-105.0,-82.8,-8.4,-17.9,-17.4\""
      << " --pos \"0.4274,0.2936,0.0997\""
      << " --quat \"0.4385,-0.5591,0.5707,-0.4116\""
      << " --elbow \"0.317,0.658,0.684\""
      << " --input-frame shoulder --dh-d1 0.1745\n";
}

Options parseArgs(int argc, char **argv)
{
  Options options;
  options.kine_config_path =
      "/home/mmlab/codes/huangshzh/ros2_control_system/ros2_control_marvin/third_party/marvinCfg/ccs_m6_40.MvKDCfg";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string &flag) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + flag);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--kine-config") {
      options.kine_config_path = require_value(arg);
    } else if (arg == "--arm") {
      options.arm = parseArm(require_value(arg));
    } else if (arg == "--ref") {
      options.ref_joint_deg = parseArray<kJoints>(require_value(arg), "--ref");
    } else if (arg == "--pos") {
      options.pos_m = parseArray<3>(require_value(arg), "--pos");
    } else if (arg == "--quat") {
      options.quat_xyzw = parseArray<4>(require_value(arg), "--quat");
    } else if (arg == "--elbow") {
      options.elbow_dir = parseArray<3>(require_value(arg), "--elbow");
    } else if (arg == "--zsp-angle") {
      options.zsp_angle_deg = std::stod(require_value(arg));
    } else if (arg == "--dh-d1") {
      options.dh_d1_m = std::stod(require_value(arg));
    } else if (arg == "--input-frame") {
      const std::string frame = trim(require_value(arg));
      if (frame == "base") {
        options.input_pos_is_shoulder_frame = false;
      } else if (frame == "shoulder") {
        options.input_pos_is_shoulder_frame = true;
      } else {
        throw std::runtime_error("Invalid --input-frame, use base|shoulder.");
      }
    } else if (arg == "--mode") {
      const std::string mode = trim(require_value(arg));
      if (mode == "near_dir") {
        options.run_near_dir = true;
        options.run_near_ref = false;
      } else if (mode == "near_ref") {
        options.run_near_dir = false;
        options.run_near_ref = true;
      } else if (mode == "both") {
        options.run_near_dir = true;
        options.run_near_ref = true;
      } else {
        throw std::runtime_error("Invalid --mode, use near_dir|near_ref|both.");
      }
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  return options;
}

std::array<double, 3> resolvedTargetPosM(const Options &options)
{
  std::array<double, 3> pos = options.pos_m;
  if (options.input_pos_is_shoulder_frame) {
    pos[2] += options.dh_d1_m;
  }
  return pos;
}

bool initMarvinKinematics(const std::string &config_path, MarvinKineData &kd)
{
  FX_LOG_SWITCH(0);

  if (!LOADMvCfg(
          const_cast<FX_CHAR *>(config_path.c_str()),
          kd.type, kd.grv, kd.dh, kd.pnva, kd.bd,
          kd.mass, kd.mcp, kd.inertia)) {
    return false;
  }

  for (int arm = 0; arm < 2; ++arm) {
    if (!FX_Robot_Init_Type(arm, kd.type[arm])) {
      return false;
    }
    if (!FX_Robot_Init_Kine(arm, kd.dh[arm])) {
      return false;
    }
    if (!FX_Robot_Init_Lmt(arm, kd.pnva[arm], kd.bd[arm])) {
      return false;
    }
  }

  return true;
}

void fillTargetMatrix(
    const std::array<double, 3> &pos_m,
    const std::array<double, 4> &quat_xyzw,
    Matrix4 target)
{
  std::memset(target, 0, sizeof(Matrix4));

  double qx = quat_xyzw[0];
  double qy = quat_xyzw[1];
  double qz = quat_xyzw[2];
  double qw = quat_xyzw[3];
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-12) {
    throw std::runtime_error("Quaternion norm is zero.");
  }
  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  const double s = 2.0;
  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;

  target[0][0] = 1.0 - s * (yy + zz);
  target[0][1] = s * (xy - wz);
  target[0][2] = s * (xz + wy);
  target[0][3] = pos_m[0] * kM2Mm;

  target[1][0] = s * (xy + wz);
  target[1][1] = 1.0 - s * (xx + zz);
  target[1][2] = s * (yz - wx);
  target[1][3] = pos_m[1] * kM2Mm;

  target[2][0] = s * (xz - wy);
  target[2][1] = s * (yz + wx);
  target[2][2] = 1.0 - s * (xx + yy);
  target[2][3] = pos_m[2] * kM2Mm;

  target[3][3] = 1.0;
}

LimitCheck checkCandidate(
    int arm, const MarvinKineData &kd, const std::array<double, kJoints> &joint_deg)
{
  LimitCheck check;
  for (size_t j = 0; j < kJoints; ++j) {
    check.run_limit_pos[j] = kd.pnva[arm][j][0];
    check.run_limit_neg[j] = kd.pnva[arm][j][1];
  }

  if (kd.type[arm] == FX_ROBOT_TYPE_PILOT_CCS) {
    const double j6 = joint_deg[5];
    if (j6 > 1.0) {
      const double upper = kd.bd[arm][0][0] * j6 * j6 + kd.bd[arm][0][1] * j6 + kd.bd[arm][0][2];
      const double lower = kd.bd[arm][3][0] * j6 * j6 + kd.bd[arm][3][1] * j6 + kd.bd[arm][3][2];
      check.run_limit_pos[6] = std::min(check.run_limit_pos[6], upper);
      check.run_limit_neg[6] = std::max(check.run_limit_neg[6], lower);
    } else if (j6 < -1.0) {
      const double upper = kd.bd[arm][1][0] * j6 * j6 + kd.bd[arm][1][1] * j6 + kd.bd[arm][1][2];
      const double lower = kd.bd[arm][2][0] * j6 * j6 + kd.bd[arm][2][1] * j6 + kd.bd[arm][2][2];
      check.run_limit_pos[6] = std::min(check.run_limit_pos[6], upper);
      check.run_limit_neg[6] = std::max(check.run_limit_neg[6], lower);
    }
  }

  for (size_t j = 0; j < kJoints; ++j) {
    if (joint_deg[j] < check.run_limit_neg[j]) {
      check.in_limits = false;
      check.exceeded[j] = true;
      check.max_exceed_abs = std::max(check.max_exceed_abs, check.run_limit_neg[j] - joint_deg[j]);
    } else if (joint_deg[j] > check.run_limit_pos[j]) {
      check.in_limits = false;
      check.exceeded[j] = true;
      check.max_exceed_abs = std::max(check.max_exceed_abs, joint_deg[j] - check.run_limit_pos[j]);
    }
  }

  return check;
}

double computeRotationErrorDeg(const Matrix4 lhs, const Matrix4 rhs)
{
  double r_rel[3][3]{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        r_rel[i][j] += lhs[k][i] * rhs[k][j];
      }
    }
  }
  double trace = r_rel[0][0] + r_rel[1][1] + r_rel[2][2];
  double cos_angle = (trace - 1.0) * 0.5;
  cos_angle = std::clamp(cos_angle, -1.0, 1.0);
  return std::acos(cos_angle) * kRad2Deg;
}

void printLimitCheck(const LimitCheck &check)
{
  if (check.in_limits) {
    std::cout << "OK";
    return;
  }

  std::cout << "EXCEEDED";
  for (size_t j = 0; j < kJoints; ++j) {
    if (!check.exceeded[j]) {
      continue;
    }
    std::cout << " J" << (j + 1)
              << "[" << check.run_limit_neg[j]
              << ", " << check.run_limit_pos[j] << "]";
  }
  std::cout << " max_exceed=" << check.max_exceed_abs << " deg";
}

void printCandidateRow(
    int arm, const MarvinKineData &kd, const Matrix4 target,
    size_t index, const std::array<double, kJoints> &joint_deg)
{
  const LimitCheck limit_check = checkCandidate(arm, kd, joint_deg);

  Matrix4 fk{};
  FX_DOUBLE joints[kJoints];
  for (size_t j = 0; j < kJoints; ++j) {
    joints[j] = joint_deg[j];
  }
  const bool fk_ok = FX_Robot_Kine_FK(arm, joints, fk);

  double pos_err_mm = 0.0;
  double rot_err_deg = 0.0;
  if (fk_ok) {
    const double dx = fk[0][3] - target[0][3];
    const double dy = fk[1][3] - target[1][3];
    const double dz = fk[2][3] - target[2][3];
    pos_err_mm = std::sqrt(dx * dx + dy * dy + dz * dz);
    rot_err_deg = computeRotationErrorDeg(fk, target);
  }

  std::cout << "  Candidate " << index << ": q_deg=[";
  for (size_t j = 0; j < kJoints; ++j) {
    if (j > 0) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(3) << joint_deg[j];
  }
  std::cout << "] | limits=";
  printLimitCheck(limit_check);
  if (fk_ok) {
    std::cout << " | fk_err=(" << std::setprecision(4) << pos_err_mm
              << " mm, " << rot_err_deg << " deg)";
  } else {
    std::cout << " | fk_err=FK failed";
  }
  std::cout << "\n";
}

void runMode(
    const std::string &label,
    int arm,
    const MarvinKineData &kd,
    const Options &options,
    FX_INT32L zsp_type)
{
  FX_InvKineSolvePara solve{};
  const auto target_pos_m = resolvedTargetPosM(options);
  fillTargetMatrix(target_pos_m, options.quat_xyzw, solve.m_Input_IK_TargetTCP);
  for (size_t j = 0; j < kJoints; ++j) {
    solve.m_Input_IK_RefJoint[j] = options.ref_joint_deg[j];
  }
  solve.m_Input_IK_ZSPType = zsp_type;
  solve.m_Input_ZSP_Angle = options.zsp_angle_deg;
  if (zsp_type == FX_PILOT_NSP_TYPES_NEAR_DIR) {
    std::array<double, 3> elbow = options.elbow_dir;
    const double norm = std::sqrt(
        elbow[0] * elbow[0] + elbow[1] * elbow[1] + elbow[2] * elbow[2]);
    if (norm < 1e-12) {
      throw std::runtime_error("Elbow direction norm is zero.");
    }
    for (double &v : elbow) {
      v /= norm;
    }
    solve.m_Input_IK_ZSPPara[0] = elbow[0];
    solve.m_Input_IK_ZSPPara[1] = elbow[1];
    solve.m_Input_IK_ZSPPara[2] = elbow[2];
  }

  const bool ok = FX_Robot_Kine_IK(arm, &solve);
  std::cout << "\n[" << label << "]\n";
  std::cout << "  return=" << (ok ? "true" : "false")
            << " result_num=" << solve.m_OutPut_Result_Num
            << " is_joint_exceeded=" << (solve.m_Output_IsJntExd ? "true" : "false")
            << " exd_abs=" << solve.m_Output_JntExdABS << " deg\n";

  std::cout << "  selected=[";
  for (size_t j = 0; j < kJoints; ++j) {
    if (j > 0) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(3) << solve.m_Output_RetJoint[j];
  }
  std::cout << "]\n";

  std::cout << "  selected_deg_flags=[";
  for (size_t j = 0; j < kJoints; ++j) {
    if (j > 0) {
      std::cout << ", ";
    }
    std::cout << (solve.m_Output_IsDeg[j] ? "1" : "0");
  }
  std::cout << "]\n";

  std::cout << "  selected_exceeded_tags=[";
  for (size_t j = 0; j < kJoints; ++j) {
    if (j > 0) {
      std::cout << ", ";
    }
    std::cout << (solve.m_Output_JntExdTags[j] ? "1" : "0");
  }
  std::cout << "]\n";

  Matrix4 target{};
  fillTargetMatrix(target_pos_m, options.quat_xyzw, target);

  const size_t count = std::min<size_t>(solve.m_OutPut_Result_Num, kMaxCandidates);
  for (size_t i = 0; i < count; ++i) {
    std::array<double, kJoints> candidate{};
    for (size_t j = 0; j < kJoints; ++j) {
      candidate[j] = solve.m_OutPut_AllJoint[i][j];
    }
    printCandidateRow(arm, kd, target, i, candidate);
  }
  for (size_t i = count; i < 4; ++i) {
    std::cout << "  Candidate " << i << ": <none>\n";
  }
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    if (argc == 1) {
      printUsage(argv[0]);
      return 1;
    }

    const Options options = parseArgs(argc, argv);

    MarvinKineData kd;
    if (!initMarvinKinematics(options.kine_config_path, kd)) {
      std::cerr << "Failed to initialize kinematics from: "
                << options.kine_config_path << "\n";
      return 2;
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Kinematics config: " << options.kine_config_path << "\n";
    std::cout << "Arm: " << options.arm << "\n";
    std::cout << "Input frame: " << (options.input_pos_is_shoulder_frame ? "shoulder" : "base") << "\n";
    std::cout << "Input pos(m): [" << options.pos_m[0] << ", " << options.pos_m[1]
              << ", " << options.pos_m[2] << "]\n";
    const auto target_pos_m = resolvedTargetPosM(options);
    std::cout << "Resolved base_T_ee pos(m): [" << target_pos_m[0] << ", " << target_pos_m[1]
              << ", " << target_pos_m[2] << "]\n";
    std::cout << "Target quat(xyzw): [" << options.quat_xyzw[0] << ", " << options.quat_xyzw[1]
              << ", " << options.quat_xyzw[2] << ", " << options.quat_xyzw[3] << "]\n";
    std::cout << "Ref joints(deg): [";
    for (size_t j = 0; j < kJoints; ++j) {
      if (j > 0) {
        std::cout << ", ";
      }
      std::cout << options.ref_joint_deg[j];
    }
    std::cout << "]\n";
    std::cout << "Elbow dir: [" << options.elbow_dir[0] << ", " << options.elbow_dir[1]
              << ", " << options.elbow_dir[2] << "]\n";
    std::cout << "ZSP angle(deg): " << options.zsp_angle_deg << "\n";
    std::cout << "dh_d1(m): " << options.dh_d1_m << "\n";

    if (options.run_near_dir) {
      runMode("NEAR_DIR", options.arm, kd, options, FX_PILOT_NSP_TYPES_NEAR_DIR);
    }
    if (options.run_near_ref) {
      runMode("NEAR_REF", options.arm, kd, options, FX_PILOT_NSP_TYPES_NEAR_REF);
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
