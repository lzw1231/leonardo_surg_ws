#pragma once

#include <cstdint>
#include <linux/can.h>

namespace psm_hardware_interface {

/// V6 从臂 CAN 协议帧构造器（仅编码，不涉及发送与 fd）。
class CanProtocol {
  public:
    /// A1：全部轴使能（Byte2=0x80，bit7=1）。
    static can_frame make_enable_all(uint32_t arm_id);

    /// A1：全部轴解除使能（Byte2=0x00，bit7=0 且 bit0~bit6 全 0）。
    static can_frame make_disable_all(uint32_t arm_id);

    /// A2：停止全部轴，按当前位置保持位置伺服（Byte2=0x01）。
    static can_frame make_stop_all(uint32_t arm_id);

    /// A2：全部轴回零（Byte2=0x02）。
    static can_frame make_home_all(uint32_t arm_id);

    /// A3：设置最大限制电流与速度等级。
    /// @param arm_id       从臂 CAN ID（0x441/0x442/0x443）
    /// @param current_01a  最大限制电流，单位 0.01A（0~255 → 0~2.55A）
    /// @param speed_level  速度等级 1/2/3（30%/60%/100% 最大速度）
    static can_frame make_set_current_speed(uint32_t arm_id, uint8_t current_01a, uint8_t speed_level);
};

} // namespace psm_hardware_interface