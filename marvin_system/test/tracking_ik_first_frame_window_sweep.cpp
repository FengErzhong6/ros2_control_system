#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "FxRobot.h"
#include "TrackingIk.h"

namespace {

constexpr size_t kArmCount = 2;
constexpr size_t kJoints = 7;
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

struct DebugFrame {
  bool has_pose{false};
  bool has_prev_ref_dir{false};
  bool has_desired_dir{false};
  bool has_baseline_result{false};
  std::array<double, 3> shoulder_pos_m{};
  std::array<double, 4> quat_xyzw{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> prev_selected_ref_dir{{0.0, 0.0, 1.0}};
  std::array<double, 3> desired_upper_arm_dir{{0.0, 0.0, 1.0}};
  int baseline_branch{-1};
  double baseline_psi_deg{0.0};
  std::array<double, kJoints> baseline_q_deg{};
  double baseline_nsp_dir_deg{0.0};
};

struct ControllerConfig {
  double fk_accept_tol{1.0e-3};
  double fine_psi_range_deg{2.0};
  double fine_psi_step_deg{0.1};
  double fast_psi_range_deg{12.0};
  double fast_psi_step_deg{0.5};
  double expand_psi_range_deg{63.0};
  double expand_psi_step_deg{2.0};
  double desired_dir_weight{0.06};
  double continuity_dir_weight{0.05};
  double magnitude_weight{0.05};
  double psi_delta_weight{0.00};
  double branch_switch_penalty{20.0};
  double dh_d1_m{0.1745};
  std::array<double, kJoints> home_left_rad{};
  std::array<double, kJoints> home_right_rad{};
};

struct SolveSummary {
  bool success{false};
  bool reachable{false};
  bool used_expanded_search{false};
  int psi_eval_count{0};
  int candidate_count{0};
  int selected_branch{-1};
  double selected_psi_deg{0.0};
  std::array<double, kJoints> q_deg{};
  std::array<double, 3> selected_ref_dir{{0.0, 0.0, 0.0}};
  bool solved_upper_arm_dir_valid{false};
  std::array<double, 3> solved_upper_arm_dir{{0.0, 0.0, 0.0}};
  double nsp_dir_deg{0.0};
};

struct SweepRow {
  double expand_range_deg{0.0};
  double expand_step_deg{0.0};
  int expand_offset_count{0};
  SolveSummary arm[2];
  bool branch_equal{false};
  double psi_gap_abs_deg{std::numeric_limits<double>::infinity()};
  double mirror_l1_deg{std::numeric_limits<double>::infinity()};
  double mirror_max_abs_deg{std::numeric_limits<double>::infinity()};
};

struct Options {
  std::string controllers_yaml;
  std::string debug_file;
  std::string kine_config;
  double range_min_deg{-1.0};
  double range_max_deg{120.0};
  double range_step_deg{1.0};
  double step_quantum_deg{0.5};
  int max_expand_offsets{64};
  std::string csv_path;
  bool has_ref_left{false};
  bool has_ref_right{false};
  std::array<double, kJoints> ref_left_deg{};
  std::array<double, kJoints> ref_right_deg{};
  bool mirror_left_from_right{false};
  bool mirror_right_from_left{false};
};

std::string trim(const std::string &value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

bool startsWith(const std::string &value, const std::string &prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::vector<double> parseList(const std::string &raw)
{
  std::string normalized = raw;
  for (char &ch : normalized) {
    if (ch == ',' || ch == '[' || ch == ']' || ch == ';') {
      ch = ' ';
    }
  }

  std::stringstream ss(normalized);
  std::vector<double> values;
  double value = 0.0;
  while (ss >> value) {
    values.push_back(value);
  }
  return values;
}

template <size_t N>
std::array<double, N> parseArray(const std::string &raw, const std::string &name)
{
  const auto values = parseList(raw);
  if (values.size() != N) {
    std::ostringstream err;
    err << "Expected " << N << " values for " << name << ", got " << values.size() << ".";
    throw std::runtime_error(err.str());
  }

  std::array<double, N> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

std::string extractBracketPayloadAfter(const std::string &line, const std::string &token)
{
  const auto pos = line.find(token);
  if (pos == std::string::npos) {
    throw std::runtime_error("Failed to find token '" + token + "' in line: " + line);
  }
  const auto begin = line.find('[', pos);
  const auto end = line.find(']', begin);
  if (begin == std::string::npos || end == std::string::npos || end <= begin) {
    throw std::runtime_error("Failed to parse bracket payload in line: " + line);
  }
  return line.substr(begin + 1, end - begin - 1);
}

double extractScalarAfter(const std::string &line, const std::string &token)
{
  const auto pos = line.find(token);
  if (pos == std::string::npos) {
    throw std::runtime_error("Failed to find token '" + token + "' in line: " + line);
  }
  const std::string tail = line.substr(pos + token.size());
  const auto values = parseList(tail);
  if (values.empty()) {
    throw std::runtime_error("Failed to parse scalar after '" + token + "' in line: " + line);
  }
  return values.front();
}

template <size_t N>
std::string formatArray(const std::array<double, N> &values, int precision = 3)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < N; ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << std::fixed << std::setprecision(precision) << values[i];
  }
  oss << "]";
  return oss.str();
}

bool normalizeVector(std::array<double, 3> &vector)
{
  const double norm = std::sqrt(
      vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  if (norm < 1e-12) {
    return false;
  }
  vector[0] /= norm;
  vector[1] /= norm;
  vector[2] /= norm;
  return true;
}

double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

double angleBetweenVectorsDeg(
    const std::array<double, 3> &lhs,
    const std::array<double, 3> &rhs)
{
  return std::acos(clampUnit(
      lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2])) * kRad2Deg;
}

void printUsage(const char *argv0)
{
  std::cout
      << "Usage:\n"
      << "  " << argv0 << " [options]\n\n"
      << "Options:\n"
      << "  --controllers-yaml path    tracker teleop controller yaml\n"
      << "  --debug-file path          first-frame debug txt\n"
      << "  --kine-config path         .MvKDCfg path\n"
      << "  --range-min deg            expand_psi_range sweep start\n"
      << "  --range-max deg            expand_psi_range sweep end\n"
      << "  --range-step deg           expand_psi_range sweep increment\n"
      << "  --step-quantum deg         auto expand_psi_step quantum\n"
      << "  --max-expand-offsets N     max expand-stage offsets, default 64\n"
      << "  --ref-left \"...\"          override left ref joints in deg\n"
      << "  --ref-right \"...\"         override right ref joints in deg\n"
      << "  --mirror-left-from-right   replace left frame/ref with strict mirror of right\n"
      << "  --mirror-right-from-left   replace right frame/ref with strict mirror of left\n"
      << "  --csv path                 write full sweep table to csv\n"
      << "  --help\n";
}

Options parseArgs(int argc, char **argv)
{
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string &flag) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + flag);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--controllers-yaml") {
      options.controllers_yaml = requireValue(arg);
    } else if (arg == "--debug-file") {
      options.debug_file = requireValue(arg);
    } else if (arg == "--kine-config") {
      options.kine_config = requireValue(arg);
    } else if (arg == "--range-min") {
      options.range_min_deg = std::stod(requireValue(arg));
    } else if (arg == "--range-max") {
      options.range_max_deg = std::stod(requireValue(arg));
    } else if (arg == "--range-step") {
      options.range_step_deg = std::stod(requireValue(arg));
    } else if (arg == "--step-quantum") {
      options.step_quantum_deg = std::stod(requireValue(arg));
    } else if (arg == "--max-expand-offsets") {
      options.max_expand_offsets = std::stoi(requireValue(arg));
    } else if (arg == "--csv") {
      options.csv_path = requireValue(arg);
    } else if (arg == "--ref-left") {
      options.ref_left_deg = parseArray<kJoints>(requireValue(arg), "--ref-left");
      options.has_ref_left = true;
    } else if (arg == "--ref-right") {
      options.ref_right_deg = parseArray<kJoints>(requireValue(arg), "--ref-right");
      options.has_ref_right = true;
    } else if (arg == "--mirror-left-from-right") {
      options.mirror_left_from_right = true;
    } else if (arg == "--mirror-right-from-left") {
      options.mirror_right_from_left = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (options.controllers_yaml.empty()) {
    throw std::runtime_error("Missing required --controllers-yaml.");
  }
  if (options.debug_file.empty()) {
    throw std::runtime_error("Missing required --debug-file.");
  }
  if (options.kine_config.empty()) {
    throw std::runtime_error("Missing required --kine-config.");
  }
  if (options.range_max_deg <= 0.0) {
    throw std::runtime_error("--range-max must be > 0.");
  }
  if (options.range_step_deg <= 0.0) {
    throw std::runtime_error("--range-step must be > 0.");
  }
  if (options.step_quantum_deg <= 0.0) {
    throw std::runtime_error("--step-quantum must be > 0.");
  }
  if (options.max_expand_offsets <= 0) {
    throw std::runtime_error("--max-expand-offsets must be > 0.");
  }
  if (options.mirror_left_from_right && options.mirror_right_from_left) {
    throw std::runtime_error("Use only one of --mirror-left-from-right or --mirror-right-from-left.");
  }

  return options;
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

  for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
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

DebugFrame parseDebugFrameBlock(const std::vector<std::string> &lines)
{
  DebugFrame frame;
  const std::regex branch_psi_regex(
      R"(branch=([+-]?\d+)\s*,\s*psi=([+-]?\d+(?:\.\d+)?)\s*deg)");

  for (const std::string &raw_line : lines) {
    const std::string line = trim(raw_line);
    if (line.find("shoulder_T_ee") != std::string::npos) {
      frame.shoulder_pos_m = parseArray<3>(extractBracketPayloadAfter(line, "pos="), "shoulder_T_ee.pos");
      frame.quat_xyzw = parseArray<4>(extractBracketPayloadAfter(line, "quat="), "shoulder_T_ee.quat");
      frame.has_pose = true;
    } else if (line.find("prev_selected_ref_dir") != std::string::npos) {
      frame.prev_selected_ref_dir = parseArray<3>(
          extractBracketPayloadAfter(line, "prev_selected_ref_dir"), "prev_selected_ref_dir");
      frame.has_prev_ref_dir = true;
    } else if (line.find("desired_upper_arm_dir") != std::string::npos) {
      frame.desired_upper_arm_dir = parseArray<3>(
          extractBracketPayloadAfter(line, "desired_upper_arm_dir"), "desired_upper_arm_dir");
      frame.has_desired_dir = true;
    } else if (line.find("branch=") != std::string::npos && line.find("psi=") != std::string::npos) {
      std::smatch match;
      if (!std::regex_search(line, match, branch_psi_regex) || match.size() != 3) {
        throw std::runtime_error("Failed to parse branch/psi line: " + line);
      }
      frame.baseline_branch = std::stoi(match[1].str());
      frame.baseline_psi_deg = std::stod(match[2].str());
      frame.has_baseline_result = true;
    } else if (line.find("q=") != std::string::npos) {
      frame.baseline_q_deg = parseArray<kJoints>(extractBracketPayloadAfter(line, "q="), "baseline q");
    } else if (line.find("nsp_dir=") != std::string::npos) {
      frame.baseline_nsp_dir_deg = extractScalarAfter(line, "nsp_dir=");
    }
  }

  if (!frame.has_pose || !frame.has_prev_ref_dir || !frame.has_desired_dir) {
    throw std::runtime_error("Debug frame is incomplete.");
  }
  normalizeVector(frame.prev_selected_ref_dir);
  normalizeVector(frame.desired_upper_arm_dir);
  return frame;
}

std::array<DebugFrame, 2> loadDebugFrames(const std::string &path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Failed to open debug file: " + path);
  }

  std::vector<std::vector<std::string>> blocks;
  std::vector<std::string> current_block;
  bool current_block_has_pose = false;
  std::string line;
  while (std::getline(input, line)) {
    const bool is_pose_line = line.find("shoulder_T_ee") != std::string::npos;
    if (is_pose_line && current_block_has_pose) {
      blocks.push_back(current_block);
      current_block.clear();
      current_block_has_pose = false;
    }
    if (!trim(line).empty()) {
      current_block.push_back(line);
      current_block_has_pose = current_block_has_pose || is_pose_line;
    }
  }
  if (!current_block.empty()) {
    blocks.push_back(current_block);
  }

  if (blocks.size() < 2) {
    throw std::runtime_error("Expected two frame blocks in debug file: " + path);
  }

  std::array<DebugFrame, 2> frames{};
  frames[0] = parseDebugFrameBlock(blocks[0]);
  frames[1] = parseDebugFrameBlock(blocks[1]);
  return frames;
}

ControllerConfig loadControllerConfig(const std::string &path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Failed to open controller yaml: " + path);
  }

  ControllerConfig config;
  bool in_tracking_ik = false;
  bool in_score = false;
  bool in_home_joint_positions = false;
  int tracking_ik_indent = -1;
  int score_indent = -1;
  int home_indent = -1;

  std::string raw_line;
  while (std::getline(input, raw_line)) {
    const std::string line = trim(raw_line);
    if (line.empty() || startsWith(line, "#")) {
      continue;
    }

    int indent = 0;
    while (indent < static_cast<int>(raw_line.size()) && raw_line[indent] == ' ') {
      ++indent;
    }

    if (line == "tracking_ik:") {
      in_tracking_ik = true;
      in_score = false;
      tracking_ik_indent = indent;
      continue;
    }
    if (line == "score:" && in_tracking_ik) {
      in_score = true;
      score_indent = indent;
      continue;
    }
    if (line == "home_joint_positions:") {
      in_home_joint_positions = true;
      home_indent = indent;
      continue;
    }

    if (in_score && indent <= score_indent) {
      in_score = false;
    }
    if (in_tracking_ik && indent <= tracking_ik_indent) {
      in_tracking_ik = false;
      in_score = false;
    }
    if (in_home_joint_positions && indent <= home_indent) {
      in_home_joint_positions = false;
    }

    auto parseScalarLine = [&](const std::string &key, double &out) {
      if (startsWith(line, key + ":")) {
        out = std::stod(trim(line.substr(key.size() + 1)));
        return true;
      }
      return false;
    };

    if (startsWith(line, "dh_d1:")) {
      config.dh_d1_m = std::stod(trim(line.substr(std::string("dh_d1").size() + 1)));
      continue;
    }

    if (in_tracking_ik && !in_score) {
      if (parseScalarLine("fk_accept_tol", config.fk_accept_tol) ||
          parseScalarLine("fine_psi_range_deg", config.fine_psi_range_deg) ||
          parseScalarLine("fine_psi_step_deg", config.fine_psi_step_deg) ||
          parseScalarLine("fast_psi_range_deg", config.fast_psi_range_deg) ||
          parseScalarLine("fast_psi_step_deg", config.fast_psi_step_deg) ||
          parseScalarLine("expand_psi_range_deg", config.expand_psi_range_deg) ||
          parseScalarLine("expand_psi_step_deg", config.expand_psi_step_deg)) {
        continue;
      }
    }

    if (in_score) {
      if (parseScalarLine("desired_dir_weight", config.desired_dir_weight) ||
          parseScalarLine("continuity_dir_weight", config.continuity_dir_weight) ||
          parseScalarLine("magnitude_weight", config.magnitude_weight) ||
          parseScalarLine("psi_delta_weight", config.psi_delta_weight) ||
          parseScalarLine("branch_switch_penalty", config.branch_switch_penalty)) {
        continue;
      }
    }

    if (in_home_joint_positions) {
      if (startsWith(line, "left:")) {
        config.home_left_rad = parseArray<kJoints>(
            extractBracketPayloadAfter(line, "left:"), "home_joint_positions.left");
      } else if (startsWith(line, "right:")) {
        config.home_right_rad = parseArray<kJoints>(
            extractBracketPayloadAfter(line, "right:"), "home_joint_positions.right");
      }
    }
  }

  return config;
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

std::array<double, 3> computeBaseTargetPosM(
    const DebugFrame &frame,
    double dh_d1_m)
{
  std::array<double, 3> pos = frame.shoulder_pos_m;
  pos[2] += dh_d1_m;
  return pos;
}

bool extractSolvedUpperArmDir(
    int arm,
    const std::array<double, kJoints> &joints_deg,
    std::array<double, 3> &upper_arm_dir)
{
  Matrix4 fk_pose{};
  Matrix3 nsp_pose{};
  FX_DOUBLE joints_deg_raw[kJoints];
  for (size_t i = 0; i < kJoints; ++i) {
    joints_deg_raw[i] = joints_deg[i];
  }

  if (!FX_Robot_Kine_FK_NSP(arm, joints_deg_raw, fk_pose, nsp_pose)) {
    return false;
  }

  upper_arm_dir = {{nsp_pose[0][1], nsp_pose[1][1], nsp_pose[2][1]}};
  return normalizeVector(upper_arm_dir);
}

tracking_ik::Request buildRequest(
    const ControllerConfig &config,
    const std::array<double, 3> &base_pos_m,
    const std::array<double, 4> &quat_xyzw,
    const std::array<double, 3> &desired_upper_arm_dir,
    const std::array<double, 3> &prev_selected_ref_dir,
    int prev_selected_branch,
    const std::array<double, kJoints> &ref_joint_deg,
    double expand_range_deg,
    double expand_step_deg)
{
  tracking_ik::Request request;
  tracking_ik::SetDefaultRequest(&request);
  fillTargetMatrix(base_pos_m, quat_xyzw, request.target_tcp);
  for (size_t i = 0; i < 3; ++i) {
    request.desired_upper_arm_dir[i] = desired_upper_arm_dir[i];
    request.prev_selected_ref_dir[i] = prev_selected_ref_dir[i];
  }
  for (size_t i = 0; i < kJoints; ++i) {
    request.ref_joint_deg[i] = ref_joint_deg[i];
  }
  request.prev_selected_branch = prev_selected_branch;
  request.fk_accept_tol = config.fk_accept_tol;
  request.fine_psi_range_deg = config.fine_psi_range_deg;
  request.fine_psi_step_deg = config.fine_psi_step_deg;
  request.fast_psi_range_deg = config.fast_psi_range_deg;
  request.fast_psi_step_deg = config.fast_psi_step_deg;
  request.expand_psi_range_deg = expand_range_deg;
  request.expand_psi_step_deg = expand_step_deg;
  request.score_params.desired_dir_weight = config.desired_dir_weight;
  request.score_params.continuity_dir_weight = config.continuity_dir_weight;
  request.score_params.magnitude_weight = config.magnitude_weight;
  request.score_params.psi_delta_weight = config.psi_delta_weight;
  request.score_params.branch_switch_penalty = config.branch_switch_penalty;
  return request;
}

int inferStartupSeedBranch(
    int arm,
    const ControllerConfig &config,
    const tracking_ik::Geometry &geometry,
    const std::array<double, kJoints> &ref_joint_deg,
    std::array<double, 3> &seeded_ref_dir,
    std::array<double, 3> &classified_ref_dir)
{
  if (!extractSolvedUpperArmDir(arm, ref_joint_deg, seeded_ref_dir)) {
    throw std::runtime_error("Failed to extract startup upper-arm direction for arm " + std::to_string(arm));
  }

  FX_DOUBLE joints_deg_raw[kJoints];
  Matrix4 fk_pose{};
  for (size_t i = 0; i < kJoints; ++i) {
    joints_deg_raw[i] = ref_joint_deg[i];
  }
  if (!FX_Robot_Kine_FK(arm, joints_deg_raw, fk_pose)) {
    throw std::runtime_error("Failed to compute startup FK for arm " + std::to_string(arm));
  }

  tracking_ik::Request request;
  tracking_ik::SetDefaultRequest(&request);
  std::memcpy(request.target_tcp, fk_pose, sizeof(Matrix4));
  for (size_t i = 0; i < 3; ++i) {
    request.desired_upper_arm_dir[i] = seeded_ref_dir[i];
    request.prev_selected_ref_dir[i] = seeded_ref_dir[i];
  }
  for (size_t i = 0; i < kJoints; ++i) {
    request.ref_joint_deg[i] = ref_joint_deg[i];
  }
  request.prev_selected_branch = -1;
  request.fk_accept_tol = config.fk_accept_tol;
  request.fine_psi_range_deg = config.fine_psi_range_deg;
  request.fine_psi_step_deg = config.fine_psi_step_deg;
  request.fast_psi_range_deg = config.fast_psi_range_deg;
  request.fast_psi_step_deg = config.fast_psi_step_deg;
  request.expand_psi_range_deg = config.expand_psi_range_deg;
  request.expand_psi_step_deg = config.expand_psi_step_deg;
  request.score_params.desired_dir_weight = config.desired_dir_weight;
  request.score_params.continuity_dir_weight = config.continuity_dir_weight;
  request.score_params.magnitude_weight = config.magnitude_weight;
  request.score_params.psi_delta_weight = config.psi_delta_weight;
  request.score_params.branch_switch_penalty = config.branch_switch_penalty;

  tracking_ik::Result result{};
  if (!tracking_ik::Solve(&geometry, &request, &result)) {
    throw std::runtime_error("tracking_ik::Solve failed during startup seed inference for arm " +
                             std::to_string(arm));
  }
  if (!result.success) {
    throw std::runtime_error("tracking_ik::Solve could not classify startup seed for arm " +
                             std::to_string(arm));
  }

  classified_ref_dir = {{result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
  normalizeVector(classified_ref_dir);
  return static_cast<int>(result.selected_branch);
}

SolveSummary solveFrame(
    int arm,
    const ControllerConfig &config,
    const tracking_ik::Geometry &geometry,
    const DebugFrame &frame,
    const std::array<double, kJoints> &ref_joint_deg,
    int prev_selected_branch,
    double expand_range_deg,
    double expand_step_deg)
{
  SolveSummary summary;
  const auto base_pos_m = computeBaseTargetPosM(frame, config.dh_d1_m);
  tracking_ik::Request request = buildRequest(
      config,
      base_pos_m,
      frame.quat_xyzw,
      frame.desired_upper_arm_dir,
      frame.prev_selected_ref_dir,
      prev_selected_branch,
      ref_joint_deg,
      expand_range_deg,
      expand_step_deg);

  tracking_ik::Result result{};
  if (!tracking_ik::Solve(&geometry, &request, &result)) {
    throw std::runtime_error("tracking_ik::Solve returned false for arm " + std::to_string(arm));
  }

  summary.success = result.success;
  summary.reachable = result.reachable;
  summary.used_expanded_search = result.used_expanded_search;
  summary.psi_eval_count = static_cast<int>(result.psi_eval_count);
  summary.candidate_count = static_cast<int>(result.candidate_count);
  summary.selected_branch = static_cast<int>(result.selected_branch);
  summary.selected_psi_deg = result.selected_psi_deg;
  summary.selected_ref_dir = {{result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
  if (summary.success) {
    for (size_t i = 0; i < kJoints; ++i) {
      summary.q_deg[i] = result.best_joints_deg[i];
    }
    std::array<double, 3> solved_dir{};
    if (extractSolvedUpperArmDir(arm, summary.q_deg, solved_dir)) {
      summary.solved_upper_arm_dir_valid = true;
      summary.solved_upper_arm_dir = solved_dir;
      summary.nsp_dir_deg = angleBetweenVectorsDeg(frame.desired_upper_arm_dir, solved_dir);
    }
  }
  return summary;
}

int buildPsiOffsetCount(
    double range_deg,
    double step_deg,
    double min_abs_deg,
    bool include_zero)
{
  if (range_deg < 0.0 || step_deg <= 0.0) {
    return 0;
  }

  int count = 0;
  if (include_zero && min_abs_deg <= 0.0) {
    count += 1;
  }

  const int level_count = static_cast<int>(range_deg / step_deg + 1e-9);
  for (int level = 1; level <= level_count; ++level) {
    const double value = step_deg * level;
    if (value <= min_abs_deg + 1e-9) {
      continue;
    }
    count += 2;
  }
  return count;
}

double computeAutoExpandStep(
    double range_deg,
    double fast_range_deg,
    double base_step_deg,
    double step_quantum_deg,
    int max_expand_offsets)
{
  double step = std::max(base_step_deg, step_quantum_deg);
  for (int guard = 0; guard < 100000; ++guard) {
    if (buildPsiOffsetCount(range_deg, step, fast_range_deg, false) <= max_expand_offsets) {
      return step;
    }
    step += step_quantum_deg;
  }
  throw std::runtime_error("Failed to compute auto expand step.");
}

std::array<double, kJoints> radiansToDegrees(const std::array<double, kJoints> &rad)
{
  std::array<double, kJoints> deg{};
  for (size_t i = 0; i < kJoints; ++i) {
    deg[i] = rad[i] * kRad2Deg;
  }
  return deg;
}

std::array<double, kJoints> buildMirrorSigns(
    const std::array<double, kJoints> &left_ref_deg,
    const std::array<double, kJoints> &right_ref_deg)
{
  std::array<double, kJoints> signs{};
  for (size_t i = 0; i < kJoints; ++i) {
    if (std::abs(left_ref_deg[i]) > 1e-9 && std::abs(right_ref_deg[i]) > 1e-9) {
      signs[i] = (right_ref_deg[i] / left_ref_deg[i] >= 0.0) ? 1.0 : -1.0;
    } else {
      signs[i] = 1.0;
    }
  }
  return signs;
}

std::array<double, 3> mirrorSagittalVector(const std::array<double, 3> &vector)
{
  return {{vector[0], -vector[1], vector[2]}};
}

std::array<double, 4> rotationMatrixToQuaternion(const double rotation[3][3])
{
  std::array<double, 4> quat{};
  const double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;

  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (rotation[2][1] - rotation[1][2]) * s;
    qy = (rotation[0][2] - rotation[2][0]) * s;
    qz = (rotation[1][0] - rotation[0][1]) * s;
  } else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2]) {
    const double s = 2.0 * std::sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]);
    qw = (rotation[2][1] - rotation[1][2]) / s;
    qx = 0.25 * s;
    qy = (rotation[0][1] + rotation[1][0]) / s;
    qz = (rotation[0][2] + rotation[2][0]) / s;
  } else if (rotation[1][1] > rotation[2][2]) {
    const double s = 2.0 * std::sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]);
    qw = (rotation[0][2] - rotation[2][0]) / s;
    qx = (rotation[0][1] + rotation[1][0]) / s;
    qy = 0.25 * s;
    qz = (rotation[1][2] + rotation[2][1]) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]);
    qw = (rotation[1][0] - rotation[0][1]) / s;
    qx = (rotation[0][2] + rotation[2][0]) / s;
    qy = (rotation[1][2] + rotation[2][1]) / s;
    qz = 0.25 * s;
  }

  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  quat[0] = qx / norm;
  quat[1] = qy / norm;
  quat[2] = qz / norm;
  quat[3] = qw / norm;
  return quat;
}

std::array<double, 4> mirrorSagittalQuaternion(const std::array<double, 4> &quat_xyzw)
{
  double rotation[3][3]{};
  const double qx = quat_xyzw[0];
  const double qy = quat_xyzw[1];
  const double qz = quat_xyzw[2];
  const double qw = quat_xyzw[3];
  const double xx = qx * qx;
  const double yy = qy * qy;
  const double zz = qz * qz;
  const double xy = qx * qy;
  const double xz = qx * qz;
  const double yz = qy * qz;
  const double wx = qw * qx;
  const double wy = qw * qy;
  const double wz = qw * qz;

  rotation[0][0] = 1.0 - 2.0 * (yy + zz);
  rotation[0][1] = 2.0 * (xy - wz);
  rotation[0][2] = 2.0 * (xz + wy);
  rotation[1][0] = 2.0 * (xy + wz);
  rotation[1][1] = 1.0 - 2.0 * (xx + zz);
  rotation[1][2] = 2.0 * (yz - wx);
  rotation[2][0] = 2.0 * (xz - wy);
  rotation[2][1] = 2.0 * (yz + wx);
  rotation[2][2] = 1.0 - 2.0 * (xx + yy);

  double mirrored[3][3]{};
  const double m[3] = {1.0, -1.0, 1.0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mirrored[i][j] = m[i] * rotation[i][j] * m[j];
    }
  }
  return rotationMatrixToQuaternion(mirrored);
}

DebugFrame mirrorSagittalFrame(const DebugFrame &src)
{
  DebugFrame dst = src;
  dst.shoulder_pos_m = mirrorSagittalVector(src.shoulder_pos_m);
  dst.quat_xyzw = mirrorSagittalQuaternion(src.quat_xyzw);
  dst.prev_selected_ref_dir = mirrorSagittalVector(src.prev_selected_ref_dir);
  dst.desired_upper_arm_dir = mirrorSagittalVector(src.desired_upper_arm_dir);
  normalizeVector(dst.prev_selected_ref_dir);
  normalizeVector(dst.desired_upper_arm_dir);
  return dst;
}

void finalizeSweepMetrics(
    SweepRow &row,
    const std::array<double, kJoints> &mirror_signs)
{
  row.branch_equal =
      row.arm[0].success && row.arm[1].success &&
      row.arm[0].selected_branch == row.arm[1].selected_branch;

  if (row.arm[0].success && row.arm[1].success) {
    row.psi_gap_abs_deg = std::abs(row.arm[0].selected_psi_deg - row.arm[1].selected_psi_deg);

    double mirror_l1 = 0.0;
    double mirror_max = 0.0;
    for (size_t i = 0; i < kJoints; ++i) {
      const double diff = std::abs(mirror_signs[i] * row.arm[0].q_deg[i] - row.arm[1].q_deg[i]);
      mirror_l1 += diff;
      mirror_max = std::max(mirror_max, diff);
    }
    row.mirror_l1_deg = mirror_l1;
    row.mirror_max_abs_deg = mirror_max;
  }
}

std::string formatPsiWithBoundaryHint(
    double psi_deg,
    double range_deg,
    double step_deg)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << psi_deg;
  if (range_deg - std::abs(psi_deg) <= step_deg + 1e-9) {
    oss << "*";
  }
  return oss.str();
}

void printSweepTable(const std::vector<SweepRow> &rows)
{
  std::cout << "\nSweep results\n";
  std::cout << "range  step offsets | left(br psi j4)         | right(br psi j4)        | eq  psi_gap  mirror_l1\n";
  for (const SweepRow &row : rows) {
    std::cout << std::setw(5) << std::fixed << std::setprecision(1) << row.expand_range_deg
              << "  " << std::setw(4) << row.expand_step_deg
              << "  " << std::setw(7) << row.expand_offset_count
              << " | ";
    if (row.arm[0].success) {
      std::cout << std::setw(2) << row.arm[0].selected_branch
                << " " << std::setw(6)
                << formatPsiWithBoundaryHint(
                       row.arm[0].selected_psi_deg,
                       row.expand_range_deg,
                       row.expand_step_deg)
                << " " << std::setw(7) << std::fixed << std::setprecision(1)
                << row.arm[0].q_deg[3];
    } else {
      std::cout << "FAIL";
      std::cout << std::string(18, ' ');
    }
    std::cout << " | ";
    if (row.arm[1].success) {
      std::cout << std::setw(2) << row.arm[1].selected_branch
                << " " << std::setw(6)
                << formatPsiWithBoundaryHint(
                       row.arm[1].selected_psi_deg,
                       row.expand_range_deg,
                       row.expand_step_deg)
                << " " << std::setw(7) << std::fixed << std::setprecision(1)
                << row.arm[1].q_deg[3];
    } else {
      std::cout << "FAIL";
      std::cout << std::string(18, ' ');
    }
    std::cout << " | "
              << std::setw(3) << (row.branch_equal ? "Y" : "N")
              << "  ";
    if (std::isfinite(row.psi_gap_abs_deg)) {
      std::cout << std::setw(7) << std::fixed << std::setprecision(1) << row.psi_gap_abs_deg
                << "  " << std::setw(9) << std::fixed << std::setprecision(1) << row.mirror_l1_deg;
    } else {
      std::cout << "   n/a      n/a";
    }
    std::cout << "\n";
  }
}

const SweepRow *findBestSymmetryRow(const std::vector<SweepRow> &rows)
{
  const SweepRow *best = nullptr;
  for (const SweepRow &row : rows) {
    if (!row.arm[0].success || !row.arm[1].success) {
      continue;
    }
    if (best == nullptr) {
      best = &row;
      continue;
    }

    const auto better = [&](const SweepRow &lhs, const SweepRow &rhs) {
      if (lhs.branch_equal != rhs.branch_equal) {
        return lhs.branch_equal && !rhs.branch_equal;
      }
      if (std::abs(lhs.psi_gap_abs_deg - rhs.psi_gap_abs_deg) > 1e-9) {
        return lhs.psi_gap_abs_deg < rhs.psi_gap_abs_deg;
      }
      if (std::abs(lhs.mirror_l1_deg - rhs.mirror_l1_deg) > 1e-9) {
        return lhs.mirror_l1_deg < rhs.mirror_l1_deg;
      }
      return lhs.expand_range_deg < rhs.expand_range_deg;
    };

    if (better(row, *best)) {
      best = &row;
    }
  }
  return best;
}

const SweepRow *findFirstRightBoundaryRelease(const std::vector<SweepRow> &rows)
{
  for (const SweepRow &row : rows) {
    if (!row.arm[1].success) {
      continue;
    }
    if (row.expand_range_deg - std::abs(row.arm[1].selected_psi_deg) > row.expand_step_deg + 1e-9) {
      return &row;
    }
  }
  return nullptr;
}

void writeCsv(const std::string &path, const std::vector<SweepRow> &rows)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Failed to open csv output: " + path);
  }

  output
      << "expand_range_deg,expand_step_deg,expand_offset_count,"
      << "left_success,left_branch,left_psi_deg,left_q1,left_q2,left_q3,left_q4,left_q5,left_q6,left_q7,left_nsp_dir_deg,"
      << "right_success,right_branch,right_psi_deg,right_q1,right_q2,right_q3,right_q4,right_q5,right_q6,right_q7,right_nsp_dir_deg,"
      << "branch_equal,psi_gap_abs_deg,mirror_l1_deg,mirror_max_abs_deg\n";

  output << std::fixed << std::setprecision(6);
  for (const SweepRow &row : rows) {
    output
        << row.expand_range_deg << ","
        << row.expand_step_deg << ","
        << row.expand_offset_count << ","
        << (row.arm[0].success ? 1 : 0) << ","
        << row.arm[0].selected_branch << ","
        << row.arm[0].selected_psi_deg;
    for (double value : row.arm[0].q_deg) {
      output << "," << value;
    }
    output << "," << row.arm[0].nsp_dir_deg
           << "," << (row.arm[1].success ? 1 : 0)
           << "," << row.arm[1].selected_branch
           << "," << row.arm[1].selected_psi_deg;
    for (double value : row.arm[1].q_deg) {
      output << "," << value;
    }
    output << "," << row.arm[1].nsp_dir_deg
           << "," << (row.branch_equal ? 1 : 0)
           << "," << row.psi_gap_abs_deg
           << "," << row.mirror_l1_deg
           << "," << row.mirror_max_abs_deg
           << "\n";
  }
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    const Options options = parseArgs(argc, argv);

    const ControllerConfig config = loadControllerConfig(options.controllers_yaml);
    std::array<DebugFrame, 2> frames = loadDebugFrames(options.debug_file);

    MarvinKineData kd;
    if (!initMarvinKinematics(options.kine_config, kd)) {
      throw std::runtime_error("Failed to initialize kinematics from: " + options.kine_config);
    }

    tracking_ik::Geometry geometry{};
    if (!tracking_ik::LoadGeometryFromMvKDCfg(options.kine_config.c_str(), &geometry)) {
      throw std::runtime_error("tracking_ik::LoadGeometryFromMvKDCfg failed for: " + options.kine_config);
    }

    const std::array<double, kJoints> default_ref_left_deg = radiansToDegrees(config.home_left_rad);
    const std::array<double, kJoints> default_ref_right_deg = radiansToDegrees(config.home_right_rad);
    const std::array<double, kJoints> ref_left_deg =
        options.has_ref_left ? options.ref_left_deg : default_ref_left_deg;
    const std::array<double, kJoints> ref_right_deg =
        options.has_ref_right ? options.ref_right_deg : default_ref_right_deg;
    std::array<double, kJoints> effective_ref_left_deg = ref_left_deg;
    std::array<double, kJoints> effective_ref_right_deg = ref_right_deg;
    const std::array<double, kJoints> mirror_signs =
        buildMirrorSigns(default_ref_left_deg, default_ref_right_deg);

    if (options.mirror_left_from_right) {
      frames[0] = mirrorSagittalFrame(frames[1]);
      for (size_t i = 0; i < kJoints; ++i) {
        effective_ref_left_deg[i] = mirror_signs[i] * effective_ref_right_deg[i];
      }
    } else if (options.mirror_right_from_left) {
      frames[1] = mirrorSagittalFrame(frames[0]);
      for (size_t i = 0; i < kJoints; ++i) {
        effective_ref_right_deg[i] = mirror_signs[i] * effective_ref_left_deg[i];
      }
    }

    std::array<double, 3> left_seed_ref_dir{};
    std::array<double, 3> right_seed_ref_dir{};
    std::array<double, 3> left_classified_ref_dir{};
    std::array<double, 3> right_classified_ref_dir{};
    const int left_prev_branch =
        inferStartupSeedBranch(
            0, config, geometry, effective_ref_left_deg, left_seed_ref_dir, left_classified_ref_dir);
    const int right_prev_branch =
        inferStartupSeedBranch(
            1, config, geometry, effective_ref_right_deg, right_seed_ref_dir, right_classified_ref_dir);

    const double range_min_deg =
        options.range_min_deg > 0.0 ? options.range_min_deg : config.expand_psi_range_deg;
    if (range_min_deg > options.range_max_deg) {
      throw std::runtime_error("--range-min must be <= --range-max.");
    }

    std::vector<SweepRow> rows;
    for (double range_deg = range_min_deg;
         range_deg <= options.range_max_deg + 1e-9;
         range_deg += options.range_step_deg) {
      SweepRow row;
      row.expand_range_deg = range_deg;
      row.expand_step_deg = computeAutoExpandStep(
          range_deg,
          config.fast_psi_range_deg,
          config.expand_psi_step_deg,
          options.step_quantum_deg,
          options.max_expand_offsets);
      row.expand_offset_count = buildPsiOffsetCount(
          row.expand_range_deg,
          row.expand_step_deg,
          config.fast_psi_range_deg,
          false);

      row.arm[0] = solveFrame(
          0,
          config,
          geometry,
          frames[0],
          effective_ref_left_deg,
          left_prev_branch,
          row.expand_range_deg,
          row.expand_step_deg);
      row.arm[1] = solveFrame(
          1,
          config,
          geometry,
          frames[1],
          effective_ref_right_deg,
          right_prev_branch,
          row.expand_range_deg,
          row.expand_step_deg);
      finalizeSweepMetrics(row, mirror_signs);
      rows.push_back(row);
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Controller yaml: " << options.controllers_yaml << "\n";
    std::cout << "Debug file:      " << options.debug_file << "\n";
    std::cout << "Kine config:     " << options.kine_config << "\n";
    std::cout << "\nCurrent controller search windows\n";
    std::cout << "  fine:   range=" << config.fine_psi_range_deg
              << " step=" << config.fine_psi_step_deg << "\n";
    std::cout << "  fast:   range=" << config.fast_psi_range_deg
              << " step=" << config.fast_psi_step_deg << "\n";
    std::cout << "  expand: range=" << config.expand_psi_range_deg
              << " step=" << config.expand_psi_step_deg
              << " offsets=" << buildPsiOffsetCount(
                     config.expand_psi_range_deg,
                     config.expand_psi_step_deg,
                     config.fast_psi_range_deg,
                     false)
              << "\n";
    std::cout << "  dh_d1:  " << config.dh_d1_m << " m\n";

    std::cout << "\nRef joints used for startup seed\n";
    std::cout << "  left  (deg): " << formatArray(effective_ref_left_deg, 3) << "\n";
    std::cout << "  right (deg): " << formatArray(effective_ref_right_deg, 3) << "\n";
    std::cout << "  inferred left  seed branch=" << left_prev_branch
              << " seed_ref=" << formatArray(left_seed_ref_dir, 3)
              << " classified_ref=" << formatArray(left_classified_ref_dir, 3) << "\n";
    std::cout << "  inferred right seed branch=" << right_prev_branch
              << " seed_ref=" << formatArray(right_seed_ref_dir, 3)
              << " classified_ref=" << formatArray(right_classified_ref_dir, 3) << "\n";

    if (options.mirror_left_from_right) {
      std::cout << "\nSynthetic input override: left frame/ref is strict sagittal mirror of right.\n";
    } else if (options.mirror_right_from_left) {
      std::cout << "\nSynthetic input override: right frame/ref is strict sagittal mirror of left.\n";
    }

    std::cout << "\nDebug first-frame inputs\n";
    std::cout << "  left  shoulder_T_ee pos=" << formatArray(frames[0].shoulder_pos_m, 4)
              << " quat=" << formatArray(frames[0].quat_xyzw, 4)
              << " prev_ref=" << formatArray(frames[0].prev_selected_ref_dir, 3)
              << " desired=" << formatArray(frames[0].desired_upper_arm_dir, 3) << "\n";
    std::cout << "  right shoulder_T_ee pos=" << formatArray(frames[1].shoulder_pos_m, 4)
              << " quat=" << formatArray(frames[1].quat_xyzw, 4)
              << " prev_ref=" << formatArray(frames[1].prev_selected_ref_dir, 3)
              << " desired=" << formatArray(frames[1].desired_upper_arm_dir, 3) << "\n";

    std::cout << "\nDebug baseline results from txt\n";
    std::cout << "  left:  branch=" << frames[0].baseline_branch
              << " psi=" << frames[0].baseline_psi_deg
              << " q=" << formatArray(frames[0].baseline_q_deg, 1)
              << " nsp_dir=" << frames[0].baseline_nsp_dir_deg << "\n";
    std::cout << "  right: branch=" << frames[1].baseline_branch
              << " psi=" << frames[1].baseline_psi_deg
              << " q=" << formatArray(frames[1].baseline_q_deg, 1)
              << " nsp_dir=" << frames[1].baseline_nsp_dir_deg << "\n";

    printSweepTable(rows);

    if (const SweepRow *best = findBestSymmetryRow(rows)) {
      std::cout << "\nBest symmetry heuristic\n";
      std::cout << "  range=" << best->expand_range_deg
                << " step=" << best->expand_step_deg
                << " offsets=" << best->expand_offset_count << "\n";
      std::cout << "  left:  branch=" << best->arm[0].selected_branch
                << " psi=" << best->arm[0].selected_psi_deg
                << " q=" << formatArray(best->arm[0].q_deg, 1)
                << " nsp_dir=" << best->arm[0].nsp_dir_deg << "\n";
      std::cout << "  right: branch=" << best->arm[1].selected_branch
                << " psi=" << best->arm[1].selected_psi_deg
                << " q=" << formatArray(best->arm[1].q_deg, 1)
                << " nsp_dir=" << best->arm[1].nsp_dir_deg << "\n";
      std::cout << "  branch_equal=" << (best->branch_equal ? "true" : "false")
                << " psi_gap_abs=" << best->psi_gap_abs_deg
                << " mirror_l1=" << best->mirror_l1_deg
                << " mirror_max=" << best->mirror_max_abs_deg << "\n";
    }

    if (const SweepRow *released = findFirstRightBoundaryRelease(rows)) {
      std::cout << "\nFirst row where right psi is no longer within one expand step of the sweep boundary\n";
      std::cout << "  range=" << released->expand_range_deg
                << " step=" << released->expand_step_deg
                << " right_psi=" << released->arm[1].selected_psi_deg << "\n";
    } else {
      std::cout << "\nRight psi stays boundary-adjacent across the scanned range.\n";
    }

    if (!options.csv_path.empty()) {
      writeCsv(options.csv_path, rows);
      std::cout << "\nCSV written to: " << options.csv_path << "\n";
    }

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
