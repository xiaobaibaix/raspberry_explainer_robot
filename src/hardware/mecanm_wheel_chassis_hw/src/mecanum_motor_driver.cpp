#include "mecanum_wheel_chassis_hw/mecanum_motor_driver.hpp"
#include <iostream>
#include <spdlog/spdlog.h>
#include <iomanip>

using namespace std::chrono;

enum RecvDataStatus
{
    DATA_hand1,
    DATA_hand2,
    DATA_equipment,
    DATA_len,
    DATA_data,
    DATA_CRC,
};

enum EquipmentType
{
    EncodeMotor = 0x04,
    None
};

enum MotorCMDType
{
    SET_pwm = 0x01,
    GET_encode = 0x04,
};

static constexpr uint8_t HEAD1 = 0xAA;
static constexpr uint8_t HEAD2 = 0x55;

/* ---------- 构造/析构 ---------- */
MecanumMotorDriver::MecanumMotorDriver(const std::string &device_name, uint32_t baud)
    : io_ctx_(2),
      driver_(io_ctx_)
{
    /* 配置串口参数 */
    drivers::serial_driver::SerialPortConfig cfg(
        baud,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    try
    {
        port_ = std::make_shared<drivers::serial_driver::SerialPort>(
            io_ctx_, device_name, cfg);

        // 打开串口
        port_->open();

        std::cout << "Serial port opened successfully: " << device_name
                  << " at " << baud << " baud" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
        throw std::runtime_error("Cannot open serial port: " + device_name + " - " + e.what());
    }

    if (!port_->is_open())
    {
        throw std::runtime_error("Cannot open serial port: " + device_name);
    }

    /* 启动解析线程 */
    thread_ = std::thread(&MecanumMotorDriver::readThreadFunc, this);
    std::cout << "Read thread started" << std::endl;
}

MecanumMotorDriver::~MecanumMotorDriver()
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
    }
    cv_.notify_all();

    if (thread_.joinable())
    {
        thread_.join();
        std::cout << "Read thread joined" << std::endl;
    }

    if (port_ && port_->is_open())
    {
        port_->close();
        std::cout << "Serial port closed" << std::endl;
    }
}

/* ---------- 公有接口 ---------- */
void MecanumMotorDriver::writeSpeed(const std::array<int16_t, 4> &pwm)
{
    auto frame = buildWriteFrame(pwm);
    try
    {
        port_->send(frame);
        // std::cout << "send speed Frame: ";
        // for (const auto &byte : frame)
        // {
        //     std::cout << std::hex << std::setfill('0') << std::setw(2)
        //               << static_cast<int>(static_cast<uint8_t>(byte)) << " ";
        // }
        // std::cout << std::dec << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Send error: " << e.what() << std::endl;
    }
}

std::array<int32_t, 4> MecanumMotorDriver::readEncoder()
{
    auto frame = buildReadFrame();
    // t=steady_clock::now();
    try
    {
        port_->send(frame);
        // std::cout << "send cmd Frame: ";
        // for (const auto &byte : frame)
        // {
        //     std::cout << std::hex << std::setfill('0') << std::setw(2)
        //               << static_cast<int>(static_cast<uint8_t>(byte)) << " ";
        // }
        // std::cout << std::dec << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Send error: " << e.what() << std::endl;
    }

    static std::array<int32_t, 4> enc{};
    if (!waitForEncoder(enc, milliseconds(5)))
    {
        std::cerr << "readEncoder timeout" << std::endl;
        return {INT32_MAX,INT32_MAX,INT32_MAX,INT32_MAX};
    }
    return enc;
}

/* ---------- 私有实现 ---------- */
std::vector<uint8_t> MecanumMotorDriver::buildWriteFrame(const std::array<int16_t, 4> &pwm)
{
    // 0xaa 0x55 motor(1) datalen(1) cmd(1) motorszie(1) | id(1) data(2) id(1) data(2) id(1) data(2) id(1) data(2) crc(1)
    std::vector<uint8_t> f;
    f.insert(f.end(), {HEAD1, HEAD2, EquipmentType::EncodeMotor});

    std::vector<uint8_t> data;
    data.push_back(MotorCMDType::SET_pwm);
    data.push_back(4);
    for (size_t i = 0; i < 4; ++i)
    {
        data.push_back(static_cast<uint8_t>(i + 1));
        data.push_back(static_cast<uint8_t>(pwm[i] & 0xFF));
        data.push_back(static_cast<uint8_t>((pwm[i] >> 8) & 0xFF));
    }
    f.push_back(data.size()+1); // 数据长度
    f.insert(f.end(),
             std::make_move_iterator(data.begin()),
             std::make_move_iterator(data.end()));
    f.push_back(addDiff({f.begin() + 2, f.end()}));
    return f;
}

std::vector<uint8_t> MecanumMotorDriver::buildReadFrame()
{
    // 0xaa 0x55 motor(1) datalen(1) cmd(1) timer(2) motorszie(1) | id(1) id(1) id(1) id(1) crc(1)
    // back :0xaa 0x55 motor(1) datalen(1) cmd(1) timer(2) motorszie(1) | id(1) data(4) id(1) data(4) id(1) data(4) id(1) data(4) crc(1)
    std::vector<uint8_t> f;
    f.insert(f.end(), {HEAD1, HEAD2, EquipmentType::EncodeMotor});
    std::vector<uint8_t> data;
    data.insert(data.end(), {MotorCMDType::GET_encode, 0x04, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00});
    f.push_back(data.size() + 1);
    f.insert(f.end(),
             std::make_move_iterator(data.begin()),
             std::make_move_iterator(data.end()));
    // 去掉头计算check
    // std::cout << "[buildReadFrame]";
    f.push_back(addDiff({f.begin() + 2, f.end()}));
    return f;
}

uint8_t MecanumMotorDriver::crc8(const std::vector<uint8_t> &data, size_t len)
{
    uint8_t crc = 0xFF;
    size_t data_len = std::min(len, data.size());
    for (size_t i = 0; i < data_len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}

uint8_t MecanumMotorDriver::addDiff(const std::vector<uint8_t> &data)
{
    // 0xff-循环相加结果取低八位数据
    if (data.empty())
        return 0x00;
    uint32_t num = 0;

    // std::cout << "[addDiff]" << "the chack datas::";
    for (auto d : data)
    {
        num += d;
        // std::cout << std::hex << std::setfill('0') << std::setw(2)
        //           << static_cast<int>(static_cast<uint8_t>(d)) << " ";
    }
    // std::cout << std::dec << std::endl;
    return (0xff - num) & 0xff;
}

/* ---------- 后台解析线程 ---------- */
void MecanumMotorDriver::readThreadFunc()
{
    std::vector<uint8_t> raw;
    raw.reserve(128);

    RecvDataStatus status = RecvDataStatus::DATA_hand1;

    std::unordered_map<EquipmentType, std::function<void(const std::vector<uint8_t> &data)>> handlers;

    handlers[EquipmentType::EncodeMotor] = [&](const std::vector<uint8_t> &data)
    {
        int count = data.at(0);
        // std::cout<< "Received Encoder motor count: " << count << std::endl;

        // for (const auto & byte : data) {
        //     std::cout << std::hex << std::setfill('0') << std::setw(2)
        //       << static_cast<int>(static_cast<uint8_t>(byte)) << " ";
        // }
        // std::cout << std::dec << std::endl;

        // auto dt=steady_clock::now()-t;

        // auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
        // std::cout << "readEncoder callback took " << duration_us << " us" << std::endl;

        static const int data_size = 4;
        for (int i = 0; i < count; i++)
        {
            int id = data.at(i * (data_size + 1) + 1);
            int32_t encode_dat = (data.at(i * (data_size + 1) + 2)) |
                                 (data.at(i * (data_size + 1) + 3) << 8) |
                                 (data.at(i * (data_size + 1) + 4) << 16) |
                                 (data.at(i * (data_size + 1) + 5) << 24);
            if (id > 4 || id < 1)
            { // 不在对应id电机范围
                continue;
            }
            else
            {
                this->encQueues_.at(id - 1).push(encode_dat);
            }
            // std::cout << "Encoder " << id
            //           << ": " << encode_dat << std::endl;
        }
    };

    std::cout << "[readThreadFunc]" << "Read thread running" << std::endl;
    int len = 0;
    while (true)
    {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            if (stop_)
                break;
        }
        /* 接收数据 */
        std::vector<uint8_t> chunk(64);
        try
        {
            size_t n = port_->receive(chunk); // 阻塞式接收
            if (n > 0)
            {
                chunk.resize(n);
                // std::cout << "[readThreadFunc]" << "fun Received " << n << " bytes: ";
                // for (const auto &byte : chunk)
                // {
                //     std::cout << std::hex << std::setfill('0') << std::setw(2)
                //               << static_cast<int>(static_cast<uint8_t>(byte)) << " ";
                // }
                // std::cout << std::dec << std::endl;
            }
            else
            {
                std::this_thread::sleep_for(milliseconds(10));
                continue;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[readThreadFunc]" << "Receive error: " << e.what() << std::endl;
            std::this_thread::sleep_for(milliseconds(10));
            continue;
        }

        /* 拆帧逻辑 */
        /* 拆帧逻辑 - 处理每个字节 */
        for (const auto &byte : chunk)
        { // 使用 const 引用
            switch (status)
            {
            case RecvDataStatus::DATA_hand1:
            {
                if (byte != HEAD1)
                {
                    std::cout << "[readThreadFunc] <hand1 err><"
                              << std::hex << std::setfill('0') << std::setw(2)
                              << static_cast<int>(byte) << ">" << std::dec << std::endl;
                    // 状态错误，重置状态
                    status = RecvDataStatus::DATA_hand1;
                }
                else
                {
                    status = RecvDataStatus::DATA_hand2;
                    // std::cout << "[readThreadFunc] <hand1 success><"
                    //           << std::hex << std::setfill('0') << std::setw(2)
                    //           << static_cast<int>(byte) << ">" << std::dec << std::endl;
                }
                break;
            }
            case RecvDataStatus::DATA_hand2:
            {
                if (byte == HEAD2)
                {
                    status = RecvDataStatus::DATA_equipment;
                }
                else
                {
                    // 帧头不匹配，重新开始
                    status = RecvDataStatus::DATA_hand1;
                }
                break;
            }
            case RecvDataStatus::DATA_equipment:
            {
                if (byte <= EquipmentType::None)
                {
                    raw.push_back(byte);
                    status = RecvDataStatus::DATA_len;
                }
                else
                {
                    // 无效设备类型，重新开始
                    status = RecvDataStatus::DATA_hand1;
                    raw.clear();
                }
                break;
            }
            case RecvDataStatus::DATA_len:
            {
                if (byte >= 28)
                { // 长度太大，可能是错误
                    status = RecvDataStatus::DATA_hand1;
                    raw.clear();
                }
                else
                {
                    raw.push_back(byte);
                    // std::cout << "[readThreadFunc] the frame len: "
                    //           << static_cast<int>(byte) << ", expected data length: "
                    //           << static_cast<int>(byte) - 1 << std::endl;
                    len = 0;
                    status = RecvDataStatus::DATA_data;
                }
                break;
            }
            case RecvDataStatus::DATA_data:
            {
                raw.push_back(byte);
                len++;

                // 注意：expected_len 是总长度（包括命令、数量、数据等）
                // 需要收到 expected_len 个字节（从设备类型之后的所有字节）
                if (len >= raw[1] - 1)
                { // 修正这里
                    status = RecvDataStatus::DATA_CRC;
                }
                break;
            }
            case RecvDataStatus::DATA_CRC:
            {
                // std::cout << "[readThreadFunc] ---------recv frame complete---------" << std::endl;
                // std::cout << "Frame data (" << raw.size() << " bytes): ";
                // for (auto c : raw)
                // {
                //     std::cout << std::hex << std::setfill('0') << std::setw(2)
                //               << static_cast<int>(c) << " ";
                // }
                // std::cout << std::dec << std::endl;

                // 计算校验和（不包括帧头 0xAA 0x55）
                uint8_t calc_crc = addDiff({raw.begin(), raw.end()});
                uint8_t recv_crc = byte;

                // std::cout << "[readThreadFunc] Calculated CRC: " << std::hex
                //           << static_cast<int>(calc_crc) << ", Received CRC: "
                //           << static_cast<int>(recv_crc) << std::dec << std::endl;

                if (calc_crc != recv_crc)
                {
                    std::cerr << "[readThreadFunc] CRC error, skipping frame. "
                              << "Calc: 0x" << std::hex << static_cast<int>(calc_crc)
                              << " Recv: 0x" << static_cast<int>(recv_crc) << std::dec
                              << " Data size: " << raw.size() << std::endl;

                    // 打印数据用于调试
                    std::cout << "Data for CRC calc: ";
                    for (size_t i = 2; i < raw.size(); ++i)
                    {
                        std::cout << std::hex << std::setfill('0') << std::setw(2)
                                  << static_cast<int>(raw[i]) << " ";
                    }
                    std::cout << std::dec << std::endl;
                }
                else
                {
                    // std::cout << "[readThreadFunc] CRC success!" << std::endl;
                    // 调用处理器（从第4个字节开始是数据）
                    if (raw.size() >= 4)
                    {
                        auto it = handlers.find(static_cast<EquipmentType>(raw[0]));
                        if (it != handlers.end())
                        {
                            // 从设备类型之后的数据（跳过帧头、设备类型、长度）
                            // raw[3] 是长度，所以数据从 raw[4] 开始
                            it->second(std::vector<uint8_t>(raw.begin() + 3, raw.end()));
                        }
                        else
                        {
                            std::cerr << "[readThreadFunc] Unknown equipment type: "
                                      << static_cast<int>(raw[2]) << std::endl;
                        }
                    }
                }

                // 处理完一帧，重置状态
                status = RecvDataStatus::DATA_hand1;
                raw.clear();
                break;
            }
            } // end switch
        } // end for

        std::this_thread::sleep_for(milliseconds(1));
    }
    std::cout << "[readThreadFunc]" << "Read thread exiting" << std::endl;
}

/* ---------- 同步等待 4 路编码器 ---------- */
bool MecanumMotorDriver::waitForEncoder(std::array<int32_t, 4> &enc, milliseconds timeout)
{
    auto deadline = steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lk(mtx_);

    bool ok = cv_.wait_until(lk, deadline, [&]{
        for (const auto & q : encQueues_) {
            if (q.empty()) return false;
        }
        return true; 
    });

    if (ok)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            enc[i] = encQueues_[i].front();
            encQueues_[i].pop();
        }
    }
    return ok;
}