#include "psm_hardware/common/can_protocol.hpp"

#include <stdexcept>

namespace psm_hardware_interface {

/// A1：全部轴使能。Byte2=0x80（bit7=1）。
can_frame CanProtocol::make_enable_all(uint32_t arm_id) {
    can_frame f{};
    f.can_id = arm_id;
    f.can_dlc = 3;
    f.data[0] = 0x01; // Byte0 指令序号
    f.data[1] = 0xA1; // Byte1 指令代码 A1
    f.data[2] = 0x80; // Byte2 轴使能位图：bit7=1 → 全部轴使能
    return f;
}

/// A1：全部轴解除使能。Byte2=0x00（bit7=0，bit0~bit6 全 0）。
can_frame CanProtocol::make_disable_all(uint32_t arm_id) {
    can_frame f{};
    f.can_id = arm_id;
    f.can_dlc = 3;
    f.data[0] = 0x01; // Byte0 指令序号
    f.data[1] = 0xA1; // Byte1 指令代码 A1
    f.data[2] = 0x00; // Byte2 轴使能位图：全 0 → 全部轴去使能
    return f;
}

/// A2：停止全部轴，按当前位置保持位置伺服。Byte2=0x01。
can_frame CanProtocol::make_stop_all(uint32_t arm_id) {
    can_frame f{};
    f.can_id = arm_id;
    f.can_dlc = 3;
    f.data[0] = 0x01; // Byte0 指令序号
    f.data[1] = 0xA2; // Byte1 指令代码 A2
    f.data[2] = 0x01; // Byte2 输入参数：0x01 停止（0x00 启动 / 0x02 回零）
    return f;
}

/// A2：全部轴回零。Byte2=0x02。
can_frame CanProtocol::make_home_all(uint32_t arm_id) {
    can_frame f{};
    f.can_id = arm_id;
    f.can_dlc = 3;
    f.data[0] = 0x01; // Byte0 指令序号
    f.data[1] = 0xA2; // Byte1 指令代码 A2
    f.data[2] = 0x02; // Byte2 输入参数：0x02 回零（0x00 启动 / 0x01 停止）
    return f;
}

/// A3：设置最大限制电流与速度等级。Byte2=速度等级，Byte3=电流（0.01A）。
/// @throws std::invalid_argument speed_level 不在 1~3 范围内。
can_frame CanProtocol::make_set_current_speed(uint32_t arm_id, uint8_t current_01a, uint8_t speed_level) {
    if (speed_level < 1 || speed_level > 3) {
        throw std::invalid_argument("速度等级必须为 1/2/3");
    }
    can_frame f{};
    f.can_id = arm_id;
    f.can_dlc = 4;
    f.data[0] = 0x01;        // Byte0 指令序号
    f.data[1] = 0xA3;        // Byte1 指令代码 A3
    f.data[2] = speed_level; // Byte2 速度等级（速度在前）
    f.data[3] = current_01a; // Byte3 最大限制电流（0.01A 单位）
    return f;
}

} // namespace psm_hardware_interface