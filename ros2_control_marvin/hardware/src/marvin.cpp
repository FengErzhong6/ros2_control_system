#include "ros2_control_marvin/marvin.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_control_marvin/MarvinSDK.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Ip4
{
	FX_UCHAR a{192}, b{168}, c{1}, d{190};
};

std::optional<Ip4> parse_ip4(const std::string &ip)
{
	int ia, ib, ic, id;
	if(std::sscanf(ip.c_str(), "%d.%d.%d.%d", &ia, &ib, &ic, &id) != 4){
		return std::nullopt;
	}
	auto in_range = [](int v) { return v >= 0 && v <= 255; };
	if(!in_range(ia) || !in_range(ib) || !in_range(ic) || !in_range(id)){
		return std::nullopt;
	}
	Ip4 out;
	out.a = static_cast<FX_UCHAR>(ia);
	out.b = static_cast<FX_UCHAR>(ib);
	out.c = static_cast<FX_UCHAR>(ic);
	out.d = static_cast<FX_UCHAR>(id);
	return out;
}

int clamp_int(int v, int lo, int hi)
{
	return std::min(hi, std::max(lo, v));
}

struct SharedSdk
{
	std::mutex api_mtx;
	std::mutex data_mtx;
	std::condition_variable cv;
	std::thread io_thread;
	std::atomic<bool> stop_requested{false};
	bool io_running{false};
	std::chrono::milliseconds io_period{4};
	std::chrono::milliseconds send_period{10};
	bool auto_clear_errors{false};
	std::chrono::milliseconds auto_clear_min_interval{1000};
	bool sdk_log_enabled{false};
	std::chrono::microseconds sdk_min_api_sleep{0};

	// Connection state (guarded by api_mtx).
	int refcount{0};
	bool connected{false};
	std::optional<Ip4> connected_ip;

	// Cached data + commands (guarded by data_mtx).
	DCSS cached_dcss{};
	bool cached_valid{false};
	std::array<int, 2> cached_frame_serial{{-1, -1}};
	std::array<Clock::time_point, 2> cached_frame_time{{Clock::now(), Clock::now()}};
	std::array<bool, 2> cached_arm_error{{false, false}};
	std::array<int, 2> cached_err_code{{0, 0}};
	std::array<int, 2> cached_state{{0, 0}};

	std::array<double, 7> cmd_a{};
	std::array<double, 7> cmd_b{};
	bool cmd_a_valid{false};
	bool cmd_b_valid{false};
	std::array<double, 7> last_sent_a{};
	std::array<double, 7> last_sent_b{};
	bool last_sent_valid{false};
	Clock::time_point last_send_time{Clock::now()};
	std::array<Clock::time_point, 2> last_auto_clear_attempt{{Clock::now() - std::chrono::hours(1), Clock::now() - std::chrono::hours(1)}};

	static SharedSdk &instance()
	{
		static SharedSdk inst;
		return inst;
	}

	void maybe_sleep_between_sdk_calls() const
	{
		if(sdk_min_api_sleep.count() > 0){
			std::this_thread::sleep_for(sdk_min_api_sleep);
		}
	}

	bool connect_locked(const Ip4 &ip, int connect_timeout_ms, rclcpp::Logger logger)
	{
		if(connected){
			if(connected_ip.has_value() &&
			   connected_ip->a == ip.a && connected_ip->b == ip.b &&
			   connected_ip->c == ip.c && connected_ip->d == ip.d){
				return true;
			}
			RCLCPP_ERROR(logger, "MarvinSDK already connected to a different IP.");
			return false;
		}

		const auto start = Clock::now();
		while(true){
			if(OnLinkTo(ip.a, ip.b, ip.c, ip.d)){
				connected = true;
				connected_ip = ip;
				{
					std::lock_guard<std::mutex> lk(data_mtx);
					cmd_a_valid = false;
					cmd_b_valid = false;
					cached_valid = false;
					last_sent_valid = false;
				}
				if(!sdk_log_enabled){
					OnLogOff();
					OnLocalLogOff();
				}
				return true;
			}

			const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
			if(elapsed.count() > connect_timeout_ms){
				RCLCPP_ERROR(logger, "OnLinkTo failed (timeout %dms).", connect_timeout_ms);
				return false;
			}

			// Best-effort backoff. This is not in the control loop.
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	void release_locked(rclcpp::Logger logger)
	{
		if(!connected){
			return;
		}
		if(!OnRelease()){
			RCLCPP_WARN(logger, "OnRelease returned false.");
		}
		connected = false;
		connected_ip.reset();
		{
			std::lock_guard<std::mutex> lk(data_mtx);
			cmd_a_valid = false;
			cmd_b_valid = false;
			cached_valid = false;
			last_sent_valid = false;
		}
	}

	bool clear_errors_and_flush_locked(rclcpp::Logger logger)
	{
		(void)logger;
		OnClearSet();
		maybe_sleep_between_sdk_calls();
		OnClearErr_A();
		maybe_sleep_between_sdk_calls();
		OnClearErr_B();
		maybe_sleep_between_sdk_calls();
		const bool ok = OnSetSend();
		// Vendor recommends waiting longer after clear error.
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		return ok;
	}

	bool set_position_mode_locked(int arm_idx, int vel_ratio, int acc_ratio)
	{
		OnClearSet();
		maybe_sleep_between_sdk_calls();
		bool ok = true;
		if(arm_idx == 0){
			ok = ok && OnSetTargetState_A(1);
			maybe_sleep_between_sdk_calls();
			ok = ok && OnSetJointLmt_A(vel_ratio, acc_ratio);
		}else{
			ok = ok && OnSetTargetState_B(1);
			maybe_sleep_between_sdk_calls();
			ok = ok && OnSetJointLmt_B(vel_ratio, acc_ratio);
		}
		ok = ok && OnSetSend();
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		return ok;
	}

	static ArmState arm_state_from_dcss(const DCSS &dcss, int arm_idx)
	{
		return static_cast<ArmState>(dcss.m_State[arm_idx].m_CurState);
	}

	static bool arm_has_error(const DCSS &dcss, int arm_idx)
	{
		return arm_state_from_dcss(dcss, arm_idx) == ARM_STATE_ERROR || dcss.m_State[arm_idx].m_ERRCode != 0;
	}

	bool get_buf_locked(DCSS *out)
	{
		if(!out){
			return false;
		}
		return OnGetBuf(out);
	}

	bool wait_for_position_state_locked(
		int arm_idx, int state_timeout_ms, rclcpp::Logger logger)
	{
		DCSS dcss{};
		const auto start = Clock::now();
		auto last_log = start;
		while(true){
			if(!OnGetBuf(&dcss)){
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			if(arm_has_error(dcss, arm_idx)){
				RCLCPP_ERROR(logger, "Arm %d error while switching mode: state=%d err=%d", arm_idx,
					static_cast<int>(arm_state_from_dcss(dcss, arm_idx)), dcss.m_State[arm_idx].m_ERRCode);
				return false;
			}
			if(arm_state_from_dcss(dcss, arm_idx) == ARM_STATE_POSITION){
				return true;
			}

			const auto now = Clock::now();
			if(now - last_log > std::chrono::seconds(1)){
				last_log = now;
				RCLCPP_WARN(logger, "Arm %d waiting for POSITION: cur_state=%d err=%d", arm_idx,
					static_cast<int>(arm_state_from_dcss(dcss, arm_idx)), dcss.m_State[arm_idx].m_ERRCode);
			}
			const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
			if(elapsed.count() > state_timeout_ms){
				RCLCPP_ERROR(logger, "Timeout waiting for ARM_STATE_POSITION on arm %d", arm_idx);
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}

	bool ensure_other_cmd_valid_locked(int other_arm_idx)
	{
		DCSS dcss{};
		if(!OnGetBuf(&dcss)){
			return false;
		}
		{
			std::lock_guard<std::mutex> lk(data_mtx);
			if(other_arm_idx == 0){
				for(int i = 0; i < 7; ++i){
					cmd_a[static_cast<size_t>(i)] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_Pos[i]);
				}
				cmd_a_valid = true;
			}else{
				for(int i = 0; i < 7; ++i){
					cmd_b[static_cast<size_t>(i)] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_Pos[i]);
				}
				cmd_b_valid = true;
			}
		}
		return true;
	}

	static bool commands_close(const std::array<double, 7> &a, const std::array<double, 7> &b, double eps)
	{
		for(size_t i = 0; i < 7; ++i){
			if(std::abs(a[i] - b[i]) > eps){
				return false;
			}
		}
		return true;
	}

	bool should_send_locked(const Clock::time_point now, double eps)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		if(!cmd_a_valid || !cmd_b_valid){
			// Don't send uninitialized commands.
			return false;
		}
		if(!last_sent_valid){
			return true;
		}
		if(now - last_send_time >= send_period){
			return true;
		}
		return !(commands_close(cmd_a, last_sent_a, eps) && commands_close(cmd_b, last_sent_b, eps));
	}

	void copy_cmd_locked(std::array<double, 7> *out_a, std::array<double, 7> *out_b)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		*out_a = cmd_a;
		*out_b = cmd_b;
	}

	void set_last_sent_locked(const std::array<double, 7> &a, const std::array<double, 7> &b)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		last_sent_a = a;
		last_sent_b = b;
		last_sent_valid = true;
		last_send_time = Clock::now();
	}

	void update_cached_locked(const DCSS &dcss)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		cached_dcss = dcss;
		cached_valid = true;
		for(int arm_idx = 0; arm_idx < 2; ++arm_idx){
			cached_err_code[arm_idx] = dcss.m_State[arm_idx].m_ERRCode;
			cached_state[arm_idx] = dcss.m_State[arm_idx].m_CurState;
			cached_arm_error[arm_idx] = arm_has_error(dcss, arm_idx);

			const int frame = dcss.m_Out[arm_idx].m_OutFrameSerial;
			if(frame != cached_frame_serial[arm_idx]){
				cached_frame_serial[arm_idx] = frame;
				cached_frame_time[arm_idx] = Clock::now();
			}
		}
	}

	bool get_cached(DCSS *out)
	{
		if(!out){
			return false;
		}
		std::lock_guard<std::mutex> lk(data_mtx);
		if(!cached_valid){
			return false;
		}
		*out = cached_dcss;
		return true;
	}

	bool cached_arm_error_for(int arm_idx, int *state_out, int *err_out)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		if(state_out) *state_out = cached_state[arm_idx];
		if(err_out) *err_out = cached_err_code[arm_idx];
		return cached_arm_error[arm_idx];
	}

	Clock::time_point cached_frame_time_for(int arm_idx)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		return cached_frame_time[arm_idx];
	}

	int cached_frame_serial_for(int arm_idx)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		return cached_frame_serial[arm_idx];
	}

	void ensure_cmds_valid_from_cached(const DCSS &dcss)
	{
		std::lock_guard<std::mutex> lk(data_mtx);
		if(!cmd_a_valid){
			for(int i = 0; i < 7; ++i){
				cmd_a[static_cast<size_t>(i)] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_Pos[i]);
			}
			cmd_a_valid = true;
		}
		if(!cmd_b_valid){
			for(int i = 0; i < 7; ++i){
				cmd_b[static_cast<size_t>(i)] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_Pos[i]);
			}
			cmd_b_valid = true;
		}
	}

	bool send_both_arms(const std::array<double, 7> &a, const std::array<double, 7> &b)
	{
		OnClearSet();
		double q_a[7];
		double q_b[7];
		for(int i = 0; i < 7; ++i){
			q_a[i] = a[static_cast<size_t>(i)];
			q_b[i] = b[static_cast<size_t>(i)];
		}
		bool ok = true;
		ok = ok && OnSetJointCmdPos_A(q_a);
		ok = ok && OnSetJointCmdPos_B(q_b);
		ok = ok && OnSetSend();
		return ok;
	}

	void start_io_if_needed(rclcpp::Logger logger)
	{
		if(io_running){
			return;
		}
		stop_requested.store(false);
		io_running = true;
		io_thread = std::thread([this, logger]() mutable {
			auto next = Clock::now();
			DCSS dcss{};
			while(!stop_requested.load()){
				next += io_period;
				{
					std::lock_guard<std::mutex> api_lock(api_mtx);
					if(!connected){
						// nothing
					}else{
						if(OnGetBuf(&dcss)){
							update_cached_locked(dcss);
							ensure_cmds_valid_from_cached(dcss);
						}
					}
				}

				// Auto clear errors (throttled). This is best-effort and off by default.
				if(auto_clear_errors){
					for(int arm_idx = 0; arm_idx < 2; ++arm_idx){
						int st = 0, err = 0;
						const bool has_err = cached_arm_error_for(arm_idx, &st, &err);
						if(!has_err){
							continue;
						}
						const auto now = Clock::now();
						{
							std::lock_guard<std::mutex> lk(data_mtx);
							if(now - last_auto_clear_attempt[arm_idx] < auto_clear_min_interval){
								continue;
							}
							last_auto_clear_attempt[arm_idx] = now;
						}
						RCLCPP_WARN(logger, "Arm %d error (state=%d err=%d). Attempting auto clear.", arm_idx, st, err);
						{
							std::lock_guard<std::mutex> api_lock(api_mtx);
							if(connected){
								clear_errors_and_flush_locked(logger);
							}
						}
					}
				}

				// Send latest command (throttled / only on change).
				const auto now = Clock::now();
				if(should_send_locked(now, 1e-6)){
					std::array<double, 7> a_cmd{};
					std::array<double, 7> b_cmd{};
					copy_cmd_locked(&a_cmd, &b_cmd);
					std::lock_guard<std::mutex> api_lock(api_mtx);
					if(connected){
						(void)send_both_arms(a_cmd, b_cmd);
						set_last_sent_locked(a_cmd, b_cmd);
					}
				}

				std::this_thread::sleep_until(next);
			}
		});
	}

	void stop_io_if_running()
	{
		if(!io_running){
			return;
		}
		stop_requested.store(true);
		if(io_thread.joinable()){
			io_thread.join();
		}
		io_running = false;
	}
};

}  // namespace

namespace ros2_control_marvin{

MarvinHardware::~MarvinHardware()
{
	// Best-effort release in case lifecycle didn't cleanup.
	auto &sdk = SharedSdk::instance();
	bool do_stop = false;
	{
		std::lock_guard<std::mutex> api_lock(sdk.api_mtx);
		if(sdk.refcount > 0){
			sdk.refcount -= 1;
			if(sdk.refcount == 0){
				do_stop = true;
			}
		}
	}
	if(do_stop){
		sdk.stop_io_if_running();
		std::lock_guard<std::mutex> api_lock(sdk.api_mtx);
		sdk.release_locked(rclcpp::get_logger("ros2_control_marvin.MarvinHardware"));
	}
}

hardware_interface::CallbackReturn MarvinHardware::on_init(
	const hardware_interface::HardwareComponentInterfaceParams &params
){
	const auto ret = hardware_interface::SystemInterface::on_init(params);
	if(ret != hardware_interface::CallbackReturn::SUCCESS){
		return ret;
	}

	joint_count_ = info_.joints.size();
	if(joint_count_ != 7){
		RCLCPP_FATAL(get_logger(), "MarvinHardware expects exactly 7 joints per instance, got %zu", joint_count_);
		return hardware_interface::CallbackReturn::ERROR;
	}

	hw_positions_.assign(joint_count_, 0.0);
	hw_velocities_.assign(joint_count_, 0.0);
	hw_efforts_.assign(joint_count_, 0.0);
	hw_position_commands_.assign(joint_count_, 0.0);
	joint_min_.assign(joint_count_, -std::numeric_limits<double>::infinity());
	joint_max_.assign(joint_count_, std::numeric_limits<double>::infinity());

	// Validate interfaces and parse limits.
	bool any_have_velocity = false;
	bool all_have_velocity = true;
	for(size_t idx = 0; idx < joint_count_; ++idx){
		const auto &joint = info_.joints[idx];
		bool has_pos_cmd = false;
		for(const auto &ci : joint.command_interfaces){
			if(ci.name == hardware_interface::HW_IF_POSITION){
				has_pos_cmd = true;
				const auto it_min = ci.parameters.find("min");
				const auto it_max = ci.parameters.find("max");
				if(it_min != ci.parameters.end()){
					try{ joint_min_[idx] = std::stod(it_min->second); } catch(...){}
				}
				if(it_max != ci.parameters.end()){
					try{ joint_max_[idx] = std::stod(it_max->second); } catch(...){}
				}
			}
		}
		if(!has_pos_cmd){
			RCLCPP_FATAL(get_logger(), "Joint '%s' missing required command interface '%s'", joint.name.c_str(),
				hardware_interface::HW_IF_POSITION);
			return hardware_interface::CallbackReturn::ERROR;
		}

		bool has_pos_state = false;
		bool has_vel_state = false;
		for(const auto &si : joint.state_interfaces){
			if(si.name == hardware_interface::HW_IF_POSITION){
				has_pos_state = true;
			}
			if(si.name == hardware_interface::HW_IF_VELOCITY){
				has_vel_state = true;
			}
		}
		if(!has_pos_state){
			RCLCPP_FATAL(get_logger(), "Joint '%s' missing required state interface '%s'", joint.name.c_str(),
				hardware_interface::HW_IF_POSITION);
			return hardware_interface::CallbackReturn::ERROR;
		}
		any_have_velocity = any_have_velocity || has_vel_state;
		all_have_velocity = all_have_velocity && has_vel_state;
	}
	if(any_have_velocity && !all_have_velocity){
		RCLCPP_WARN(get_logger(), "Some joints have velocity state but not all; disabling velocity state export.");
	}
	has_velocity_state_ = all_have_velocity;

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_configure(
	const rclcpp_lifecycle::State &
){
	activated_ = false;

	// Params
	std::string ip_str{"192.168.1.190"};
	{
		const auto it = info_.hardware_parameters.find("ip");
		if(it != info_.hardware_parameters.end()){
			ip_str = it->second;
		}
	}
	const auto ip = parse_ip4(ip_str);
	if(!ip.has_value()){
		RCLCPP_ERROR(get_logger(), "Invalid ip parameter: '%s'", ip_str.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}

	std::string side;
	{
		const auto it = info_.hardware_parameters.find("side");
		if(it != info_.hardware_parameters.end()){
			side = it->second;
		}
	}

	std::string sdk_arm;
	{
		const auto it = info_.hardware_parameters.find("sdk_arm");
		if(it != info_.hardware_parameters.end()){
			sdk_arm = it->second;
		}
	}
	if(sdk_arm.empty()){
		// Default mapping (override with sdk_arm if your wiring differs).
		if(side == "left") sdk_arm = "A";
		else if(side == "right") sdk_arm = "B";
		else sdk_arm = "A";
	}

	int arm_idx = 0;
	if(sdk_arm == "A" || sdk_arm == "a"){
		arm_idx = 0;
	}else if(sdk_arm == "B" || sdk_arm == "b"){
		arm_idx = 1;
	}else{
		RCLCPP_ERROR(get_logger(), "Invalid sdk_arm parameter '%s' (use A or B)", sdk_arm.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	arm_idx_ = arm_idx;

	int vel_ratio = 30;
	int acc_ratio = 30;
	int connect_timeout_ms = 1500;
	int state_timeout_ms = 2000;
	int no_frame_timeout_ms = 800;
	int mode_retry_count = 3;
	int sdk_send_period_ms = 10;
	int sdk_io_period_ms = 4;
	int sdk_auto_clear_errors = 0;
	int sdk_log_enabled = 0;
	int sdk_min_api_sleep_us = 0;
	{
		const auto it = info_.hardware_parameters.find("joint_vel_ratio");
		if(it != info_.hardware_parameters.end()) vel_ratio = clamp_int(std::atoi(it->second.c_str()), 1, 100);
	}
	{
		const auto it = info_.hardware_parameters.find("joint_acc_ratio");
		if(it != info_.hardware_parameters.end()) acc_ratio = clamp_int(std::atoi(it->second.c_str()), 1, 100);
	}
	{
		const auto it = info_.hardware_parameters.find("connect_timeout_ms");
		if(it != info_.hardware_parameters.end()) connect_timeout_ms = std::max(100, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("state_timeout_ms");
		if(it != info_.hardware_parameters.end()) state_timeout_ms = std::max(100, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("no_frame_timeout_ms");
		if(it != info_.hardware_parameters.end()) no_frame_timeout_ms = std::max(50, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("mode_retry_count");
		if(it != info_.hardware_parameters.end()) mode_retry_count = std::max(0, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("sdk_send_period_ms");
		if(it != info_.hardware_parameters.end()) sdk_send_period_ms = std::max(1, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("sdk_io_period_ms");
		if(it != info_.hardware_parameters.end()) sdk_io_period_ms = std::max(1, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("sdk_auto_clear_errors");
		if(it != info_.hardware_parameters.end()) sdk_auto_clear_errors = std::max(0, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("sdk_log_enabled");
		if(it != info_.hardware_parameters.end()) sdk_log_enabled = std::max(0, std::atoi(it->second.c_str()));
	}
	{
		const auto it = info_.hardware_parameters.find("sdk_min_api_sleep_us");
		if(it != info_.hardware_parameters.end()) sdk_min_api_sleep_us = std::max(0, std::atoi(it->second.c_str()));
	}
	no_frame_timeout_ms_ = no_frame_timeout_ms;

	// Acquire SDK session
	auto &sdk = SharedSdk::instance();
	{
		std::lock_guard<std::mutex> api_lock(sdk.api_mtx);
		// Configure shared SDK behavior on first user.
		if(sdk.refcount == 0){
			sdk.io_period = std::chrono::milliseconds(sdk_io_period_ms);
			sdk.send_period = std::chrono::milliseconds(sdk_send_period_ms);
			sdk.auto_clear_errors = (sdk_auto_clear_errors != 0);
			sdk.sdk_log_enabled = (sdk_log_enabled != 0);
			sdk.sdk_min_api_sleep = std::chrono::microseconds(sdk_min_api_sleep_us);
		}
		sdk.refcount += 1;
		if(!sdk.connect_locked(*ip, connect_timeout_ms, get_logger())){
			sdk.refcount -= 1;
			return hardware_interface::CallbackReturn::ERROR;
		}
		sdk.start_io_if_needed(get_logger());
		// Clear errors once after connect (best-effort).
		sdk.clear_errors_and_flush_locked(get_logger());

		bool mode_ok = false;
		for(int attempt = 0; attempt <= mode_retry_count; ++attempt){
			const int wait_ms = std::min(8000, state_timeout_ms * (attempt + 1));
			if(!sdk.set_position_mode_locked(arm_idx_, vel_ratio, acc_ratio)){
				RCLCPP_ERROR(get_logger(), "Failed to request position mode (arm %d), attempt %d/%d", arm_idx_, attempt + 1, mode_retry_count + 1);
			}else if(sdk.wait_for_position_state_locked(arm_idx_, wait_ms, get_logger())){
				mode_ok = true;
				break;
			}else{
				RCLCPP_ERROR(get_logger(), "Position mode not ready (arm %d), attempt %d/%d", arm_idx_, attempt + 1, mode_retry_count + 1);
			}

			// Between attempts, try clear errors and wait a bit.
			sdk.clear_errors_and_flush_locked(get_logger());
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}

		if(!mode_ok){
			RCLCPP_ERROR(get_logger(), "Unable to enter position mode (arm %d) after %d attempts", arm_idx_, mode_retry_count + 1);
			sdk.refcount -= 1;
			if(sdk.refcount == 0){
				sdk.release_locked(get_logger());
			}
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	// Initialize hold command to current position (safe default).
	DCSS dcss{};
	// Wait briefly for cached data from I/O thread.
	const auto start = Clock::now();
	while(!sdk.get_cached(&dcss)){
		if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count() > 500){
			RCLCPP_ERROR(get_logger(), "Timed out waiting for initial SDK data.");
			return hardware_interface::CallbackReturn::ERROR;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	for(size_t i = 0; i < joint_count_; ++i){
		hw_positions_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Pos[i]);
		hw_velocities_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Vel[i]);
		hw_position_commands_[i] = hw_positions_[i];
	}
	{
		std::lock_guard<std::mutex> lk(sdk.data_mtx);
		if(arm_idx_ == 0){
			for(int i = 0; i < 7; ++i) sdk.cmd_a[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_a_valid = true;
		}else{
			for(int i = 0; i < 7; ++i) sdk.cmd_b[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_b_valid = true;
		}
	}
	last_frame_serial_ = dcss.m_Out[arm_idx_].m_OutFrameSerial;
	last_frame_time_ = Clock::now();

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_activate(
	const rclcpp_lifecycle::State &
){
	activated_ = true;

	// Safety: hold current position on activate (I/O thread will send).
	DCSS dcss{};
	auto &sdk = SharedSdk::instance();
	if(!sdk.get_cached(&dcss)){
		RCLCPP_ERROR(get_logger(), "No cached SDK data during activate.");
		activated_ = false;
		return hardware_interface::CallbackReturn::ERROR;
	}
	for(size_t i = 0; i < joint_count_; ++i){
		hw_positions_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Pos[i]);
		hw_velocities_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Vel[i]);
		hw_position_commands_[i] = std::min(joint_max_[i], std::max(joint_min_[i], hw_positions_[i]));
	}
	{
		std::lock_guard<std::mutex> lk(sdk.data_mtx);
		if(arm_idx_ == 0){
			for(int i = 0; i < 7; ++i) sdk.cmd_a[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_a_valid = true;
		}else{
			for(int i = 0; i < 7; ++i) sdk.cmd_b[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_b_valid = true;
		}
	}
	last_frame_serial_ = dcss.m_Out[arm_idx_].m_OutFrameSerial;
	last_frame_time_ = Clock::now();

	for(size_t i = 0; i < joint_count_; ++i){
		const auto &jn = info_.joints[i].name;
		set_state(jn + "/" + hardware_interface::HW_IF_POSITION, hw_positions_[i]);
		set_command(jn + "/" + hardware_interface::HW_IF_POSITION, hw_position_commands_[i]);
		if(has_velocity_state_){
			set_state(jn + "/" + hardware_interface::HW_IF_VELOCITY, hw_velocities_[i]);
		}
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
	const rclcpp_lifecycle::State &
){
	// Best-effort: update hold command (I/O thread will send).
	DCSS dcss{};
	auto &sdk = SharedSdk::instance();
	if(sdk.get_cached(&dcss)){
		for(size_t i = 0; i < joint_count_; ++i){
			hw_positions_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Pos[i]);
			hw_position_commands_[i] = std::min(joint_max_[i], std::max(joint_min_[i], hw_positions_[i]));
		}
		std::lock_guard<std::mutex> lk(sdk.data_mtx);
		if(arm_idx_ == 0){
			for(int i = 0; i < 7; ++i) sdk.cmd_a[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_a_valid = true;
		}else{
			for(int i = 0; i < 7; ++i) sdk.cmd_b[static_cast<size_t>(i)] = hw_position_commands_[static_cast<size_t>(i)];
			sdk.cmd_b_valid = true;
		}
	}

	activated_ = false;
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
	const rclcpp_lifecycle::State &
){
	activated_ = false;

	auto &sdk = SharedSdk::instance();
	bool do_stop = false;
	{
		std::lock_guard<std::mutex> api_lock(sdk.api_mtx);
		if(sdk.refcount > 0){
			sdk.refcount -= 1;
			if(sdk.refcount == 0){
				do_stop = true;
			}
		}
	}
	if(do_stop){
		sdk.stop_io_if_running();
		std::lock_guard<std::mutex> api_lock(sdk.api_mtx);
		sdk.release_locked(get_logger());
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MarvinHardware::read(
	const rclcpp::Time &, const rclcpp::Duration &
){
	if(!activated_){
		return hardware_interface::return_type::OK;
	}

	auto &sdk = SharedSdk::instance();
	DCSS dcss{};
	if(!sdk.get_cached(&dcss)){
		return hardware_interface::return_type::ERROR;
	}

	int cur_state = 0;
	int err_code = 0;
	if(sdk.cached_arm_error_for(arm_idx_, &cur_state, &err_code)){
		const auto now = Clock::now();
		if(now - last_error_log_time_ > std::chrono::seconds(1)){
			last_error_log_time_ = now;
			RCLCPP_ERROR(get_logger(), "Arm %d error: state=%d err=%d", arm_idx_, cur_state, err_code);
		}
		return hardware_interface::return_type::ERROR;
	}

	const int frame = dcss.m_Out[arm_idx_].m_OutFrameSerial;
	if(frame != last_frame_serial_){
		last_frame_serial_ = frame;
		last_frame_time_ = Clock::now();
	}

	const auto no_frame_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_frame_time_);
	if(no_frame_ms.count() > no_frame_timeout_ms_){
		RCLCPP_ERROR(get_logger(), "No frame update for %ldms on arm %d", static_cast<long>(no_frame_ms.count()), arm_idx_);
		return hardware_interface::return_type::ERROR;
	}

	for(size_t i = 0; i < joint_count_; ++i){
		hw_positions_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Pos[i]);
		hw_velocities_[i] = static_cast<double>(dcss.m_Out[arm_idx_].m_FB_Joint_Vel[i]);

		const auto &jn = info_.joints[i].name;
		set_state(jn + "/" + hardware_interface::HW_IF_POSITION, hw_positions_[i]);
		if(has_velocity_state_){
			set_state(jn + "/" + hardware_interface::HW_IF_VELOCITY, hw_velocities_[i]);
		}
	}

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarvinHardware::write(
	const rclcpp::Time &, const rclcpp::Duration &
){
	if(!activated_){
		return hardware_interface::return_type::OK;
	}

	// Read desired commands from ros2_control command interfaces.
	std::array<double, 7> q_cmd{};
	for(size_t i = 0; i < joint_count_; ++i){
		const auto &jn = info_.joints[i].name;
		double cmd = get_command<double>(jn + "/" + hardware_interface::HW_IF_POSITION);
		if(!std::isfinite(cmd)){
			cmd = hw_positions_[i];
		}
		cmd = std::min(joint_max_[i], std::max(joint_min_[i], cmd));
		hw_position_commands_[i] = cmd;
		q_cmd[i] = cmd;
	}

	auto &sdk = SharedSdk::instance();
	{
		std::lock_guard<std::mutex> lk(sdk.data_mtx);
		if(arm_idx_ == 0){
			sdk.cmd_a = q_cmd;
			sdk.cmd_a_valid = true;
		}else{
			sdk.cmd_b = q_cmd;
			sdk.cmd_b_valid = true;
		}
	}

	return hardware_interface::return_type::OK;
}

} // namespace ros2_control_marvin

PLUGINLIB_EXPORT_CLASS(
	ros2_control_marvin::MarvinHardware, hardware_interface::SystemInterface
)
