#include "ros2_control_wujihand/rl_policy.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace{
    static void ensure_file_exists(const std::string &path){
        if(!std::filesystem::exists(path)){
            throw std::runtime_error("ONNX model file does not exist at path: " + path);
        }
    }
}

namespace ros2_control_wujihand{
RLPolicy::RLPolicy(const std::string &model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "RLPolicy"), session_options_(), session_(nullptr)
{
    ensure_file_exists(model_path);

    // Keep ORT threading predictable inside a realtime-ish control loop.
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = Ort::Session(env_, model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;

    // Cache input/output names and validate model I/O.
    {
        auto input_name_alloc = session_.GetInputNameAllocated(0, allocator);
        input_name_ = input_name_alloc.get();

        Ort::TypeInfo input_type_info = session_.GetInputTypeInfo(0);
        auto input_ti = input_type_info.GetTensorTypeAndShapeInfo();
        if(input_ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT){
            throw std::runtime_error("Expected float input tensor");
        }
        const auto shape = input_ti.GetShape();
        if(shape.size() != 2){
            throw std::runtime_error("Expected rank-2 input tensor shape [1,40]");
        }
        if(shape[0] > 0 && shape[0] != 1){
            throw std::runtime_error("Expected input batch dim == 1");
        }
        if(shape[1] > 0 && shape[1] != static_cast<int64_t>(kInputDim)){
            throw std::runtime_error("Expected input dim == 40");
        }
    }
    {
        auto output_name_alloc = session_.GetOutputNameAllocated(0, allocator);
        output_name_ = output_name_alloc.get();

        Ort::TypeInfo output_type_info = session_.GetOutputTypeInfo(0);
        auto output_ti = output_type_info.GetTensorTypeAndShapeInfo();
        if(output_ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT){
            throw std::runtime_error("Expected float output tensor");
        }
        const auto shape = output_ti.GetShape();
        if(shape.size() != 2){
            throw std::runtime_error("Expected rank-2 output tensor shape [1,20]");
        }
        if(shape[0] > 0 && shape[0] != 1){
            throw std::runtime_error("Expected output batch dim == 1");
        }
        if(shape[1] > 0 && shape[1] != static_cast<int64_t>(kOutputDim)){
            throw std::runtime_error("Expected output dim == 20");
        }
    }

    input_data_.fill(0.0f);
    output_data_.fill(0.0f);

    // Pre-allocate CPU tensors backed by member arrays.
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    input_tensor_ = Ort::Value::CreateTensor<float>(
        memory_info_, input_data_.data(), input_data_.size(), input_shape_.data(), input_shape_.size());
    output_tensor_ = Ort::Value::CreateTensor<float>(
        memory_info_, output_data_.data(), output_data_.size(), output_shape_.data(), output_shape_.size());

    // Cache bindings so infer() has no allocations.
    io_binding_ = std::make_unique<Ort::IoBinding>(session_);
    io_binding_->BindInput(input_name_.c_str(), input_tensor_);
    io_binding_->BindOutput(output_name_.c_str(), output_tensor_);
}

void RLPolicy::infer(const std::array<float, kInputDim> &input, std::array<float, kOutputDim> &output)
{
    std::memcpy(input_data_.data(), input.data(), sizeof(float) * kInputDim);
    session_.Run(run_options_, *io_binding_);
    std::memcpy(output.data(), output_data_.data(), sizeof(float) * kOutputDim);
}

} // namespace ros2_control_wujihand