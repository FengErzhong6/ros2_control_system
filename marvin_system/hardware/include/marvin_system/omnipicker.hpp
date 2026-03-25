#ifndef OMNIPICKER_SDK_HPP
#define OMNIPICKER_SDK_HPP

#include <cstdint>

#ifdef _WIN32
    #ifdef OMNIPICKER_BUILD_SHARED
        #define OMNIPICKER_API __declspec(dllexport)
    #else
        #define OMNIPICKER_API __declspec(dllimport)
    #endif
#else
    #define OMNIPICKER_API __attribute__((visibility("default")))
#endif

namespace omnipicker {

// ============================================================
//  错误码 —— 对应上行状态帧 D0 (Fault Code)
// ============================================================
enum class FaultCode : uint8_t {
    kNone           = 0x00,   // 无故障
    kOverTemp       = 0x01,   // 过温警报
    kOverSpeed      = 0x02,   // 超速警报
    kInitFault      = 0x03,   // 初始化故障警报
    kOverLimit      = 0x04,   // 超限检测警报
};

// ============================================================
//  运动状态 —— 对应上行状态帧 D1 (State)
// ============================================================
enum class MotionState : uint8_t {
    kReached        = 0x00,   // 已到达目标位置
    kMoving         = 0x01,   // 夹爪移动中
    kStalled        = 0x02,   // 夹爪堵转（夹住物体）
    kObjectDropped  = 0x03,   // 物体掉落
};

// ============================================================
//  夹爪状态反馈 —— 解析上行状态帧
// ============================================================
struct GripperStatus {
    FaultCode   fault    = FaultCode::kNone;
    MotionState state    = MotionState::kReached;
    uint8_t     position = 0;       // 当前位置  0x00(夹紧) ~ 0xFF(完全张开)
    uint8_t     velocity = 0;       // 当前速度  0x00 ~ 0xFF
    uint8_t     force    = 0;       // 当前力矩  0x00 ~ 0xFF
    bool        valid    = false;   // 是否已收到过有效回报
};

// ============================================================
//  控制参数 —— 组装下行控制帧
// ============================================================
struct ControlParam {
    uint8_t position     = 0x00;   // 目标位置      0x00(夹紧) ~ 0xFF(完全张开)
    uint8_t velocity     = 0xFF;   // 目标速度      0xFF = 最大速度
    uint8_t force        = 0xFF;   // 目标力矩      0xFF = 最大力矩
    uint8_t acceleration = 0xFF;   // 目标加速度    0xFF = 最大加速度
    uint8_t deceleration = 0xFF;   // 目标减速度    0xFF = 最大减速度
};

// ============================================================
//  机械臂选择
// ============================================================
enum class ArmSide : uint8_t {
    kA = 0,     // 左臂（OnSetChDataA / OnGetChDataA）
    kB = 1,     // 右臂（OnSetChDataB / OnGetChDataB）
};

// ============================================================
//  CAN 总线配置
// ============================================================
struct CANConfig {
    const char* interface   = "192.168.1.190"; // Marvin 控制器 IP (a.b.c.d)
    uint32_t    node_id     = 0x01;         // CAN 节点 ID（可用于扩展帧）
    ArmSide     arm         = ArmSide::kB;  // 夹爪安装在哪条臂上
    uint32_t    arb_bitrate = 1000000;      // 仲裁域波特率 1M
    uint32_t    data_bitrate= 5000000;      // 数据域波特率 5M (CAN-FD)
    bool        use_fd      = true;         // 是否使用 CAN-FD，false 则退化为经典 CAN
    bool        manage_link = false;        // 是否由 SDK 管理控制器连接（OnLinkTo/OnRelease）
};

// ============================================================
//  SDK 返回码
// ============================================================
enum class ErrorCode : int {
    kOK              =  0,
    kNotConnected    = -1,
    kConnectionFail  = -2,
    kSendFail        = -3,
    kRecvTimeout     = -4,
    kInvalidParam    = -5,
    kAlreadyConnected= -6,
    kInternalError   = -99,
};

/// 将 ErrorCode 转换为可读的错误描述字符串
inline const char* error_to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::kOK:               return "OK";
        case ErrorCode::kNotConnected:     return "Not connected";
        case ErrorCode::kConnectionFail:   return "Connection failed";
        case ErrorCode::kSendFail:         return "CAN frame send failed";
        case ErrorCode::kRecvTimeout:      return "Receive timeout";
        case ErrorCode::kInvalidParam:     return "Invalid parameter";
        case ErrorCode::kAlreadyConnected: return "Already connected";
        case ErrorCode::kInternalError:    return "Internal error";
        default:                           return "Unknown error";
    }
}

// ============================================================
//  IOmniPicker —— 纯虚类接口
// ============================================================
class IOmniPicker {
public:
    virtual ~IOmniPicker() = default;

    // ---- 连接管理 ----
    virtual ErrorCode connect(const CANConfig& config)  = 0;
    virtual void      disconnect()                      = 0;
    virtual bool      is_connected() const              = 0;

    /// 便捷连接：常用参数前置，均有默认值
    ErrorCode connect(ArmSide     arm         = ArmSide::kB,
                      uint32_t    node_id     = 0x01,
                      const char* interface   = "192.168.1.190",
                      bool        manage_link = false) {
        CANConfig cfg;
        cfg.arm          = arm;
        cfg.node_id      = node_id;
        cfg.interface    = interface;
        cfg.manage_link  = manage_link;
        return connect(cfg);
    }

    /// 清除伺服错误（夹爪卡死/故障后调用）
    virtual ErrorCode clear_fault()                     = 0;

    // ---- 运动控制 ----

    /// 发送完整控制指令（位置 + 速度 + 力矩 + 加减速度）
    virtual ErrorCode move(const ControlParam& param)   = 0;

    /// 设置目标位置 (0x00 ~ 0xFF)，其余参数使用默认最大值
    virtual ErrorCode set_position(uint8_t pos)         = 0;

    /// 设置目标位置 (百分比 0.0 ~ 1.0)，其余参数使用默认最大值
    virtual ErrorCode set_position_percent(float percent) = 0;

    /// 完全张开
    virtual ErrorCode open()                            = 0;

    /// 完全闭合
    virtual ErrorCode close()                           = 0;

    // ---- 状态读取 ----

    /// 主动查询：重发上一帧控制指令以触发回报，适合空闲时调用
    virtual ErrorCode query_states(GripperStatus& states) = 0;

    /// 读取缓存（完整结构体），无 CAN 通信
    virtual GripperStatus get_states()          const = 0;
    virtual uint8_t       get_position()        const = 0;
    virtual float         get_position_percent() const = 0;
    virtual uint8_t       get_velocity()        const = 0;
    virtual uint8_t       get_force()           const = 0;
    virtual FaultCode     get_fault()           const = 0;
    virtual MotionState   get_motion_state()    const = 0;

    // ---- 参数配置 ----

    /// 设置默认速度 (后续 set_position / open / close 使用)
    virtual void set_default_velocity(uint8_t vel)      = 0;

    /// 设置默认力矩
    virtual void set_default_force(uint8_t force)       = 0;

    /// 设置默认加速度
    virtual void set_default_acceleration(uint8_t acc)  = 0;

    /// 设置默认减速度
    virtual void set_default_deceleration(uint8_t dec)  = 0;

    /// 设置 query_states 的接收超时时间 (毫秒)
    virtual void set_recv_timeout_ms(uint32_t ms)       = 0;
};

}  // namespace omnipicker

// ============================================================
//  工厂函数 —— extern "C" 保证 ABI 兼容
// ============================================================
extern "C" {
    OMNIPICKER_API omnipicker::IOmniPicker* omnipicker_create();
    OMNIPICKER_API void                     omnipicker_destroy(omnipicker::IOmniPicker* p);
    OMNIPICKER_API const char*              omnipicker_version();
}

#endif  // OMNIPICKER_SDK_HPP
