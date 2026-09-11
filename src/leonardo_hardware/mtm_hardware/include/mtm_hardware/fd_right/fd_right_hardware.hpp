#pragma once
#include "mtm_hardware/fd_base/fd_hardware_base.hpp"

namespace fd_right_hardware{
    /**
     * @brief Force Dimension Right 硬件接口实现
     *
     * 继承自 FDHardwareBase，具体实现了右侧力反馈设备的 read() 和 write() 逻辑。
     */
    class FDRightHardwareInterface : public fd_hardware_base::FDHardwareBase {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDRightHardwareInterface);

        ~FDRightHardwareInterface() override;

        /**
         * @brief 读取设备状态
         *
         * 从 DHD SDK 获取位置、速度、力、惯性矩阵及按钮状态，并更新至内部缓冲区。
         * @param time 当前时间
         * @param period 周期时长
         * @return OK 或 ERROR
         */
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        /**
         * @brief 写入控制命令
         *
         * 将计算得到的力/力矩命令发送至 DHD 设备。
         * @param time 当前时间
         * @param period 周期时长
         * @return OK
         */
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    };
} // namespace fd_right_hardware
