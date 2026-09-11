#include <fd_vendor/fd_sdk.hpp>
#include <sts_vendor/SCServo.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <array>

int main() {
    int32_t id_left = dhdOpenSerial(40619);
    int32_t id_right = dhdOpenSerial(40819);

    std::cout << "========================================" << std::endl;
    std::cout << "Force‑Dimension device open status" << std::endl;
    std::cout << "  Left  (SN:40619) | id = " << id_left << (id_left >= 0 ? " | SUCCESS" : " | FAILED") << std::endl;
    std::cout << "  Right (SN:40819) | id = " << id_right << (id_right >= 0 ? " | SUCCESS" : " | FAILED") << std::endl;
    std::cout << "========================================" << std::endl;

    if (id_left < 0 || id_right < 0) {
        std::cerr << "力反馈设备打开失败，请检查连接！" << std::endl;
        return -1;
    } else {
        dhdStop(static_cast<char>(id_left));
        dhdStop(static_cast<char>(id_right));
        dhdSleep(0.1);
        int connectionIsClosed_left = dhdClose(static_cast<char>(id_left));
        int connectionIsClosed_right = dhdClose(static_cast<char>(id_right));

        if (connectionIsClosed_left >= 0 && connectionIsClosed_right >= 0) {
            std::cout << "力反馈设备已断开连接！" << std::endl;
        } else {
            std::cerr << "力反馈设备断开失败!" << std::endl;
        }
    }

    // ST3215舵机控制
    SMS_STS sms_sts;
    const char* port = "/dev/ttyACM0"; // 根据实际修改
    int baud = 1000000;

    if
    (
        !
        sms_sts
        .
        begin(baud, port)
    ) {
        std::cerr << "串口初始化失败" << std::endl;
        return -1;
    }
    std::cout
        <<
        "串口已打开"
        <<
        std::endl;

    // 6个舵机ID 11~16
    std::array<uint8_t, 6> motor_ids = {11, 12, 13, 14, 15, 16};
    // Ping检测所有舵机
    for
    (
        auto id : motor_ids
    ) {
        uint8_t ping_id = sms_sts.Ping(id);
        if (ping_id != id) {
            std::cerr << "舵机ID " << (int)id << " Ping失败" << std::endl;
            sms_sts.end();
            return -1;
        }
        std::cout << "舵机ID " << (int)id << " 在线" << std::endl;
    }

    // 设为位置模式
    for
    (
        auto id : motor_ids
    ) {
        sms_sts.ServoMode(id);
    }
    std::cout
        <<
        "所有舵机已设为位置模式"
        <<
        std::endl;

    // 准备同步写数据
    std::array<uint16_t, 6> positions;
    std::array<uint16_t, 6> speeds; // 速度
    std::array<uint8_t, 6> accs; // 加速度
    speeds
        .
        fill(
            0
        ); // 默认速度
    accs
        .
        fill(
            0
        ); // 默认加速度

    // 第一次同步写：全部转到135°
    positions
        .
        fill(
            4095
        );
    sms_sts
        .
        SyncWritePosEx(motor_ids
                       .
                       data(), motor_ids
                       .
                       size(),

                       reinterpret_cast
                       <
                           int16_t*>
                       (positions
                           .
                           data()
                       )
                       ,
                       reinterpret_cast
                       <
                           uint16_t*>
                       (speeds
                           .
                           data()
                       )
                       ,
                       accs
                       .
                       data()
        );
    std::cout
        <<
        "已发送指令：所有舵机转到360°"
        <<
        std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds
        (
            2000
        )
    ); // 等待转动

    // 第二次同步写：全部转回0°
    positions
        .
        fill(
            0
        );
    sms_sts
        .
        SyncWritePosEx(motor_ids
                       .
                       data(), motor_ids
                       .
                       size(),

                       reinterpret_cast
                       <
                           int16_t*>
                       (positions
                           .
                           data()
                       )
                       ,
                       reinterpret_cast
                       <
                           uint16_t*>
                       (speeds
                           .
                           data()
                       )
                       ,
                       accs
                       .
                       data()
        );
    std::cout
        <<
        "已发送指令：所有舵机转回0°"
        <<
        std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds
        (
            500
        )
    );


    sms_sts
        .
        end();
    std::cout
        <<
        "测试完成"
        <<
        std::endl;
    return
        0;
}
