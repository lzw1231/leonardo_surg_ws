#include <fd_vendor/fd_sdk.hpp>
#include <sts_vendor/SCServo.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <array>

int main() {
    int32_t id_40619 = dhdOpenSerial(40619);
    int32_t id_40819 = dhdOpenSerial(40819);

    std::cout << "========================================" << std::endl;
    std::cout << "Force Dimension device open status" << std::endl;

    std::cout << "  SN:40619 | id = " << id_40619;
    if (id_40619 >= 0) {
        const char* name_40619 = dhdGetSystemName(static_cast<char>(id_40619));
        std::cout << " | SUCCESS | name = " << (name_40619 ? name_40619 : "(null)");
    } else {
        std::cout << " | FAILED";
    }
    std::cout << std::endl;

    std::cout << "  SN:40819 | id = " << id_40819;
    if (id_40819 >= 0) {
        const char* name_40819 = dhdGetSystemName(static_cast<char>(id_40819));
        std::cout << " | SUCCESS | name = " << (name_40819 ? name_40819 : "(null)");
    } else {
        std::cout << " | FAILED";
    }
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;

    if (id_40619 < 0 || id_40819 < 0) {
        std::cerr << "Failed to open one or both DHD devices" << std::endl;
        return -1;
    } else {
        dhdStop(static_cast<char>(id_40619));
        dhdStop(static_cast<char>(id_40819));
        dhdSleep(0.1);
        int connectionIsClosed_40619 = dhdClose(static_cast<char>(id_40619));
        int connectionIsClosed_40819 = dhdClose(static_cast<char>(id_40819));

        if (connectionIsClosed_40619 >= 0 && connectionIsClosed_40819 >= 0) {
            std::cout << "Both DHD devices closed successfully" << std::endl;
        } else {
            std::cerr << "Failed to close one or both DHD devices" << std::endl;
        }
    }

    // ST3215 servo bus
    SMS_STS sms_sts;
    const char* port = "/dev/ttyACM0";
    int baud = 1000000;

    if (!sms_sts.begin(baud, port)) {
        std::cerr << "Failed to open serial port" << std::endl;
        return -1;
    }
    std::cout << "Serial port opened" << std::endl;

    // 6 servos, IDs 11~16
    std::array<uint8_t, 6> motor_ids = {11, 12, 13, 14, 15, 16};

    // Ping all servos
    for (auto id : motor_ids) {
        uint8_t ping_id = sms_sts.Ping(id);
        if (ping_id != id) {
            std::cerr << "Ping failed for servo ID " << (int)id << std::endl;
            sms_sts.end();
            return -1;
        }
        std::cout << "Servo ID " << (int)id << " online" << std::endl;
    }

    // Set all servos to position mode
    for (auto id : motor_ids) {
        sms_sts.ServoMode(id);
    }
    std::cout << "All servos set to position mode" << std::endl;

    std::array<uint16_t, 6> positions;
    std::array<uint16_t, 6> speeds;
    std::array<uint8_t, 6> accs;
    speeds.fill(0);
    accs.fill(0);

    // Move all servos to 4095
    positions.fill(4095);
    sms_sts.SyncWritePosEx(motor_ids.data(), motor_ids.size(),
                           reinterpret_cast<int16_t*>(positions.data()),
                           reinterpret_cast<uint16_t*>(speeds.data()),
                           accs.data());
    std::cout << "Command sent: all servos to 4095" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Move all servos back to 0
    positions.fill(0);
    sms_sts.SyncWritePosEx(motor_ids.data(), motor_ids.size(),
                           reinterpret_cast<int16_t*>(positions.data()),
                           reinterpret_cast<uint16_t*>(speeds.data()),
                           accs.data());
    std::cout << "Command sent: all servos to 0" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    sms_sts.end();
    std::cout << "Test finished" << std::endl;
    return 0;
}
