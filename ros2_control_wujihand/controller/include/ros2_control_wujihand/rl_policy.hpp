#ifndef ROS2_CONTROL_WUJIHAND__CONTROLLER__RL_POLICY_HPP_
#define ROS2_CONTROL_WUJIHAND__CONTROLLER__RL_POLICY_HPP_

#include <onnxruntime_cxx_api.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace ros2_control_wujihand{
class RLPolicy{
public:
    static constexpr size_t kInputDim = 40;
    static constexpr size_t kOutputDim = 20;

    explicit RLPolicy(const std::string &model_path);

    void infer(const std::array<float, kInputDim> &input, std::array<float, kOutputDim> &output);
    
private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};

    Ort::MemoryInfo memory_info_{nullptr};
    Ort::RunOptions run_options_{};

    std::string input_name_;
    std::string output_name_;

    std::array<int64_t, 2> input_shape_{{1, static_cast<int64_t>(kInputDim)}};
    std::array<int64_t, 2> output_shape_{{1, static_cast<int64_t>(kOutputDim)}};

    std::array<float, kInputDim> input_data_{};
    std::array<float, kOutputDim> output_data_{};

    Ort::Value input_tensor_{nullptr};
    Ort::Value output_tensor_{nullptr};

    // Cached binding to avoid per-infer allocations.
    std::unique_ptr<Ort::IoBinding> io_binding_;
};

} // namespace ros2_control_wujihand

#endif  // ROS2_CONTROL_WUJIHAND__CONTROLLER__RL_POLICY_HPP_