#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <cstring>
#include <array>
#include <algorithm>

#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"

class MecanumMotorDriver
{
public:
    explicit MecanumMotorDriver(const std::string &device_name, uint32_t baud = 115200);
    ~MecanumMotorDriver();

    void writeSpeed(const std::array<int16_t, 4> &pwm);
    std::array<int32_t, 4> readEncoder();

private:
    void readThreadFunc();
    std::vector<uint8_t> buildWriteFrame(const std::array<int16_t, 4> &pwm);
    std::vector<uint8_t> buildReadFrame();
    bool waitForEncoder(std::array<int32_t, 4> &enc, std::chrono::milliseconds timeout);
    static uint8_t crc8(const std::vector<uint8_t> &data, size_t len);
    static uint8_t addDiff(const std::vector<uint8_t> &data);

    /* 成员变量 */
    drivers::common::IoContext io_ctx_; // 使用完整的命名空间
    drivers::serial_driver::SerialDriver driver_;
    std::shared_ptr<drivers::serial_driver::SerialPort> port_;

    std::thread thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_ = false;

    std::array<std::queue<int32_t>, 4> encQueues_;
    std::chrono::_V2::steady_clock::time_point t;
};