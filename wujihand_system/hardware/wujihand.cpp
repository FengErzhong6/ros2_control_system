#include "wujihand_system/wujihand.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/filter/low_pass.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
inline constexpr size_t flat_index(const size_t finger_idx, const size_t joint_idx)
{
    // Wuji hand is fixed 5x4 (20 joints). Keep this helper local to avoid depending on
    // private class constants.
    return finger_idx * 4U + joint_idx;
}

std::string trim_copy(std::string value)
{
    const auto is_not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(
        value.begin(),
        std::find_if(value.begin(), value.end(), is_not_space)
    );
    value.erase(
        std::find_if(value.rbegin(), value.rend(), is_not_space).base(),
        value.end()
    );
    return value;
}

std::string to_lower_copy(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); }
    );
    return value;
}

const char *handedness_to_string(const uint8_t handedness)
{
    switch(handedness){
        case 0:
            return "right";
        case 1:
            return "left";
        default:
            return "unknown";
    }
}
}  // namespace

namespace wujihand_system{

WujiHandHardware::~WujiHandHardware()
{
    io_stop_.store(true, std::memory_order_relaxed);
    io_cv_.notify_all();
    if(io_thread_.joinable()){
        io_thread_.join();
    }
}

hardware_interface::CallbackReturn WujiHandHardware::submit_io_request(IoRequestType type)
{
    IoRequest req{type, std::promise<hardware_interface::CallbackReturn>{}};
    auto fut = req.promise.get_future();
    {
        std::lock_guard<std::mutex> lock(io_mutex_); // use lock_guard to avoid deadlock on exception
        if(io_request_.has_value()){
            RCLCPP_ERROR(get_logger(), "I/O thread request already pending.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        io_request_.emplace(std::move(req));
    }
    io_cv_.notify_all();
    return fut.get();
}

void WujiHandHardware::io_thread_main()
{
    using namespace std::chrono;

    // Default to 100 Hz if rw_rate is not set.
    // const int rate_hz = (info_.rw_rate > 0) ? info_.rw_rate : 100;
    const int rate_hz = 500;
    const auto period = nanoseconds(static_cast<long long>(1e9 / rate_hz));

    while(!io_stop_.load(std::memory_order_relaxed)){
        // Wait for either a request or the next periodic tick.
        std::optional<IoRequest> req;
        {
            std::unique_lock<std::mutex> lock(io_mutex_);
            io_cv_.wait_for(lock, period, [&] {
                return io_stop_.load(std::memory_order_relaxed) || io_request_.has_value();
            });

            if(io_stop_.load(std::memory_order_relaxed)){
                break;
            }

            if(io_request_.has_value()){
                req = std::move(io_request_);
                io_request_.reset();
            }
        }

        // Handle request first (synchronous lifecycle semantics).
        if(req.has_value()){
            hardware_interface::CallbackReturn result = hardware_interface::CallbackReturn::ERROR;
            try{
                switch(req->type){
                    case IoRequestType::Configure: {
                        io_error_.store(false, std::memory_order_relaxed);
                        controller_.reset();
                        hand_ = usb_serial_number_.empty()
                            ? std::make_unique<wujihandcpp::device::Hand>()
                            : std::make_unique<wujihandcpp::device::Hand>(usb_serial_number_.c_str());

                        const auto actual_handedness =
                            hand_->read<wujihandcpp::data::hand::Handedness>();
                        RCLCPP_INFO(
                            get_logger(),
                            "Configured Wuji hand: usb_serial_number=%s handedness=%s(%u)",
                            usb_serial_number_.empty() ? "<auto>" : usb_serial_number_.c_str(),
                            handedness_to_string(actual_handedness),
                            static_cast<unsigned int>(actual_handedness)
                        );

                        if(
                            expected_handedness_.has_value()
                            && actual_handedness != expected_handedness_.value()
                        ){
                            RCLCPP_ERROR(
                                get_logger(),
                                "Connected hand handedness mismatch: expected=%s(%u), actual=%s(%u).",
                                handedness_to_string(expected_handedness_.value()),
                                static_cast<unsigned int>(expected_handedness_.value()),
                                handedness_to_string(actual_handedness),
                                static_cast<unsigned int>(actual_handedness)
                            );
                            hand_.reset();
                            result = hardware_interface::CallbackReturn::ERROR;
                            break;
                        }
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Activate: {
                        io_error_.store(false, std::memory_order_relaxed);
                        if(!hand_){
                            RCLCPP_ERROR(get_logger(), "I/O Activate: hand_ is null.");
                            result = hardware_interface::CallbackReturn::ERROR;
                            break;
                        }

                        // enable all joints
                        hand_->write<wujihandcpp::data::joint::Enabled>(true);

                        // create the realtime controller
                        controller_ = hand_->realtime_controller<true>(
                            wujihandcpp::filter::LowPass{low_pass_cutoff_frequency_}
                        );
                        if(!controller_){
                            RCLCPP_ERROR(get_logger(), "I/O Activate: failed to get realtime controller.");
                            result = hardware_interface::CallbackReturn::ERROR;
                            break;
                        }

                        // Initialize state caches to current position and command caches to
                        // the configured startup pose when provided.
                        const auto &actual_pos = controller_->get_joint_actual_position();
                        double hold_target[kFingerCount][kJointCount] = {};

                        // Initialize both buffers so there is never an uninitialized frame.
                        state_frame_id_ = 0;
                        const uint8_t init_idx = 0;
                        state_active_.store(init_idx, std::memory_order_relaxed);
                        cmd_active_.store(init_idx, std::memory_order_relaxed);

                        for(size_t f = 0; f < kFingerCount; ++f){
                            for(size_t j = 0; j < kJointCount; ++j){
                                const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                                const size_t idx = flat_index(f, j);

                                const double cmd = std::isfinite(initial_command_[idx])
                                    ? initial_command_[idx]
                                    : pos;

                                state_frames_[0].data[idx] = pos;
                                state_frames_[1].data[idx] = pos;
                                cmd_frames_[0].data[idx] = cmd;
                                cmd_frames_[1].data[idx] = cmd;

                                hold_target[f][j] = cmd;
                            }
                        }
                        state_frames_[0].frame_id = 0;
                        state_frames_[1].frame_id = 0;
                        cmd_frames_[0].frame_id = 0;
                        cmd_frames_[1].frame_id = 0;

                        controller_->set_joint_target_position(hold_target);
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Deactivate: {
                        io_error_.store(false, std::memory_order_relaxed);
                        if(hand_ && controller_){
                            const auto &actual_pos = controller_->get_joint_actual_position();
                            double hold_target[kFingerCount][kJointCount] = {};

                            const uint8_t cur_s = state_active_.load(std::memory_order_relaxed);
                            const uint8_t back_s = cur_s ^ 1U;
                            const uint8_t cur_c = cmd_active_.load(std::memory_order_relaxed);
                            const uint8_t back_c = cur_c ^ 1U;

                            for(size_t f = 0; f < kFingerCount; ++f){
                                for(size_t j = 0; j < kJointCount; ++j){
                                    const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                                    const size_t idx = flat_index(f, j);
                                    state_frames_[back_s].data[idx] = pos;
                                    cmd_frames_[back_c].data[idx] = pos;

                                    hold_target[f][j] = pos;
                                }
                            }

                            state_frames_[back_s].frame_id = ++state_frame_id_;
                            state_active_.store(back_s, std::memory_order_release);

                            cmd_frames_[back_c].frame_id =
                                cmd_frame_id_.fetch_add(1, std::memory_order_relaxed) + 1;
                            cmd_active_.store(back_c, std::memory_order_release);

                            controller_->set_joint_target_position(hold_target);
                        }
                        if(hand_){
                            hand_->write<wujihandcpp::data::joint::Enabled>(false);
                        }
                        controller_.reset();
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Cleanup: {
                        io_error_.store(false, std::memory_order_relaxed);
                        try{
                            if(hand_){
                                hand_->write<wujihandcpp::data::joint::Enabled>(false);
                            }
                        } catch(const std::exception &e){
                            // Best-effort; proceed to release resources.
                            RCLCPP_WARN(get_logger(), "I/O Cleanup: disable failed: %s", e.what());
                        }
                        controller_.reset();
                        hand_.reset();
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(get_logger(), "I/O thread request exception: %s", e.what());
                result = hardware_interface::CallbackReturn::ERROR;
            }

            req->promise.set_value(result);
            continue;
        }

        // Periodic I/O while activated.
        if(!activated_.load(std::memory_order_relaxed)){
            continue;
        }
        if(io_error_.load(std::memory_order_relaxed)){
            continue;
        }
        if(!hand_ || !controller_){
            continue;
        }

        try{
            const auto &actual_pos = controller_->get_joint_actual_position();

            // Publish a coherent state frame.
            const uint8_t cur_s = state_active_.load(std::memory_order_relaxed);
            const uint8_t back_s = cur_s ^ 1U;
            // state_frames_[back_s] = state_frames_[cur_s];
            for(size_t f = 0; f < kFingerCount; ++f){
                for(size_t j = 0; j < kJointCount; ++j){
                    state_frames_[back_s].data[flat_index(f, j)] =
                        actual_pos[f][j].load(std::memory_order_relaxed);
                }
            }
            state_frames_[back_s].frame_id = ++state_frame_id_;
            state_active_.store(back_s, std::memory_order_release);

            // Read a coherent command frame.
            const uint8_t cmd_idx = cmd_active_.load(std::memory_order_acquire);
            double target[kFingerCount][kJointCount] = {};
            for(size_t f = 0; f < kFingerCount; ++f){
                for(size_t j = 0; j < kJointCount; ++j){
                    target[f][j] = cmd_frames_[cmd_idx].data[flat_index(f, j)];
                }
            }
            controller_->set_joint_target_position(target);
        } catch(const std::exception &e){
            io_error_.store(true, std::memory_order_relaxed);
            RCLCPP_ERROR(get_logger(), "I/O cycle exception: %s", e.what());
        }
    }
}

hardware_interface::CallbackReturn WujiHandHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params
){
    if(hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }
    // If we need to read some parameters from the ros2_control URDF, please add the code here.

    // Read low-pass filter cutoff from ros2_control URDF <param>.
    // Example:
    // <param name="low_pass_cutoff_frequency">5.0</param>
    low_pass_cutoff_frequency_ = 5.0;
    usb_serial_number_.clear();
    expected_handedness_.reset();
    {
        const auto it = info_.hardware_parameters.find("low_pass_cutoff_frequency");
        if(it != info_.hardware_parameters.end()){
            try{
                low_pass_cutoff_frequency_ = std::stod(it->second);
            } catch(const std::exception &e){
                RCLCPP_WARN(
                    get_logger(),
                    "Invalid low_pass_cutoff_frequency='%s', fallback to %.3f. (%s)",
                    it->second.c_str(),
                    low_pass_cutoff_frequency_,
                    e.what()
                );
            }
        }
    }
    {
        const auto it = info_.hardware_parameters.find("usb_serial_number");
        if(it != info_.hardware_parameters.end()){
            usb_serial_number_ = trim_copy(it->second);
        }
    }
    {
        const auto it = info_.hardware_parameters.find("expected_handedness");
        if(it != info_.hardware_parameters.end()){
            const auto normalized = to_lower_copy(trim_copy(it->second));
            if(normalized.empty()){
                expected_handedness_.reset();
            } else if(normalized == "right" || normalized == "0"){
                expected_handedness_ = 0;
            } else if(normalized == "left" || normalized == "1"){
                expected_handedness_ = 1;
            } else {
                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid expected_handedness='%s'. Supported values: left, right, 0, 1.",
                    it->second.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    // Log the info about the hw here
    RCLCPP_INFO(get_logger(), "The update rate of wujihand controller manager is %dHz.", info_.rw_rate);
    RCLCPP_INFO(get_logger(), "Using low_pass_cutoff_frequency = %.3f", low_pass_cutoff_frequency_);
    if(!usb_serial_number_.empty()){
        RCLCPP_INFO(get_logger(), "Using usb_serial_number = %s", usb_serial_number_.c_str());
    } else {
        RCLCPP_WARN(
            get_logger(),
            "No usb_serial_number configured. Real hardware startup requires a unique connected Wuji hand."
        );
    }
    if(expected_handedness_.has_value()){
        RCLCPP_INFO(
            get_logger(),
            "Using expected_handedness = %s",
            handedness_to_string(expected_handedness_.value())
        );
    }

    // check joint configurations in URDF
    if(info_.joints.size() != kTotalJoints){
        RCLCPP_FATAL(
            get_logger(),
            "Expected exactly %zu joints in ros2_control description, got %zu.",
            kTotalJoints,
            info_.joints.size()
        );
        return hardware_interface::CallbackReturn::ERROR;
    }

    const double neg_inf = -std::numeric_limits<double>::infinity();
    const double pos_inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    joint_min_.fill(neg_inf);
    joint_max_.fill(pos_inf);
    initial_command_.fill(nan);

    bool all_have_velocity = true;
    bool any_have_velocity = false;
    for(size_t index = 0; index < info_.joints.size(); ++index){
        const auto &joint = info_.joints[index];
        joint_names_[index] = joint.name;
        position_interface_names_[index] =
            joint.name + "/" + hardware_interface::HW_IF_POSITION;
        velocity_interface_names_[index] =
            joint.name + "/" + hardware_interface::HW_IF_VELOCITY;

        // wujihand has exactly one state and command interface on each joint
        if(joint.command_interfaces.size() != 1){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto &command_interface = joint.command_interfaces[0];
        if(command_interface.name != hardware_interface::HW_IF_POSITION){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has '%s' command interface. '%s' expected.",
                joint.name.c_str(),
                command_interface.name.c_str(),
                hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto min_it = command_interface.parameters.find("min");
        if(min_it != command_interface.parameters.end()){
            joint_min_[index] = std::stod(min_it->second);
        }
        const auto max_it = command_interface.parameters.find("max");
        if(max_it != command_interface.parameters.end()){
            joint_max_[index] = std::stod(max_it->second);
        }
        const auto initial_it = command_interface.parameters.find("initial_value");
        if(initial_it != command_interface.parameters.end()){
            initial_command_[index] = std::clamp(
                std::stod(initial_it->second),
                joint_min_[index],
                joint_max_[index]
            );
        }

        bool has_position = false;
        bool has_velocity = false;
        for(const auto &si : joint.state_interfaces){
            if(si.name == hardware_interface::HW_IF_POSITION){
                has_position = true;
            } else if(si.name == hardware_interface::HW_IF_VELOCITY){
                has_velocity = true;
            } else {
                RCLCPP_FATAL(
                    get_logger(),
                    "Joint '%s' has unsupported state interface '%s'.",
                    joint.name.c_str(),
                    si.name.c_str()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        if(!has_position){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' is missing required state interface '%s'.",
                joint.name.c_str(),
                hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        any_have_velocity = any_have_velocity || has_velocity;
        all_have_velocity = all_have_velocity && has_velocity;
    }

    if(any_have_velocity && !all_have_velocity){
        RCLCPP_WARN(
            get_logger(),
            "Some joints declare '%s' state interface but not all. Velocity state will be disabled.",
            hardware_interface::HW_IF_VELOCITY
        );
    }
    has_velocity_state_ = all_have_velocity;
    last_position_valid_ = false;

    // Start the dedicated I/O thread once. It will be responsible for ALL SDK calls.
    if(!io_thread_.joinable()){
        io_stop_.store(false, std::memory_order_relaxed);
        io_thread_ = std::thread(&WujiHandHardware::io_thread_main, this);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_configure(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    return submit_io_request(IoRequestType::Configure);
}

hardware_interface::CallbackReturn WujiHandHardware::on_activate(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);

    const auto ret = submit_io_request(IoRequestType::Activate);
    if(ret != hardware_interface::CallbackReturn::SUCCESS){
        return ret;
    }

    // Align command/state interfaces to the cached initial positions.
    const uint8_t sidx = state_active_.load(std::memory_order_acquire);
    const uint8_t cidx = cmd_active_.load(std::memory_order_acquire);
    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            const size_t flat = flat_index(f, j);
            const double pos = state_frames_[sidx].data[flat];
            const double cmd = cmd_frames_[cidx].data[flat];
            set_state(position_interface_names_[flat], pos);
            set_command(position_interface_names_[flat], cmd);

            last_position_[flat] = pos;
            if(has_velocity_state_){
                set_state(velocity_interface_names_[flat], 0.0);
            }
        }
    }

    last_position_valid_ = true;

    activated_.store(true, std::memory_order_relaxed);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_deactivate(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    last_position_valid_ = false;
    return submit_io_request(IoRequestType::Deactivate);
}

hardware_interface::CallbackReturn WujiHandHardware::on_cleanup(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    last_position_valid_ = false;
    return submit_io_request(IoRequestType::Cleanup);
}

hardware_interface::return_type WujiHandHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &period
){
    if(!activated_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::OK;
    }
    if(io_error_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::ERROR;
    }

    const uint8_t idx = state_active_.load(std::memory_order_acquire);
    const auto &frame = state_frames_[idx];

    const double dt = period.seconds();
    const bool can_diff = has_velocity_state_ && last_position_valid_ && dt > 0.0;
    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            const size_t flat = flat_index(f, j);
            const double pos = frame.data[flat];
            set_state(position_interface_names_[flat], pos);

            if(has_velocity_state_){
                const double vel = can_diff ? ((pos - last_position_[flat]) / dt) : 0.0;
                set_state(velocity_interface_names_[flat], vel);
            }

            last_position_[flat] = pos;
        }
    }

    last_position_valid_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WujiHandHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &
){
    if(!activated_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::OK;
    }
    if(io_error_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::ERROR;
    }

    const uint8_t cur = cmd_active_.load(std::memory_order_relaxed);
    const uint8_t back = cur ^ 1U;
    // Write a full coherent command frame, then publish it with a release store.
    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            const size_t flat = flat_index(f, j);
            cmd_frames_[back].data[flat] =
                get_command<double>(position_interface_names_[flat]);
        }
    }
    cmd_frames_[back].frame_id = cmd_frame_id_.fetch_add(1, std::memory_order_relaxed) + 1;
    cmd_active_.store(back, std::memory_order_release);
    return hardware_interface::return_type::OK;
}

}// namespace wujihand_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    wujihand_system::WujiHandHardware, hardware_interface::SystemInterface
)
