#include "rs485_port_manager/RS485Provider.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace rs485_port_manager
{

    RS485Provider::RS485Provider()
        : Node("rs485_provider"), _rs485Connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        _subscriptionRS485 = this->create_subscription<sonia_common_ros2::msg::RS485msg>(
            "/rs485/msgToSend", 10, std::bind(&RS485Provider::messageRS485CallBack, this, _1), sub_opt);

        _publisherKill = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/killMessage", 10);
        _publisherIO = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/ioMessage", 10);
        _publisherMotor = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/motorMessage", 10);
    }

    void RS485Provider::Start()
    {
        _reader = std::thread(std::bind(&RS485Provider::readData, this));
        _writer = std::thread(std::bind(&RS485Provider::writeData, this));
        _parser = std::thread(std::bind(&RS485Provider::parseData, this));
    }

    bool RS485Provider::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    void RS485Provider::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg)
    {
        queueObject ser;
        ser.cmd = msg.cmd;
        ser.slave = msg.slave;
        ser.data = msg.data;

        _writerQueue.push_back(ser);
        _cvReaderWriter.notify_all();
    }

    std::tuple<uint8_t, uint8_t> RS485Provider::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte,
                                                         std::vector<uint8_t> data)
    {
        uint16_t check = static_cast<uint16_t>(_START_BYTE + slave + cmd + nbByte + _END_BYTE);
        for (uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }
        return {check >> 8, check & 0XFF};
    }

    void RS485Provider::Kill()
    {
        try
        {
            _rs485Connection.Flush();
            _thread_control = false;
            _mtxWriter.unlock();
            _mtxParser.unlock();
            _writerQueue.push_back(queueObject());
            _cvReaderWriter.notify_all();
            _cvReaderParser.notify_all();
        }
        catch (error_t)
        {
            printf("Something went wrong\n");
        }
    }


    void RS485Provider::readData()
    {
        // Delay for port opening
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        sonia_common_cpp::SerialTram tram;
        tram.size = _DATA_READ_CHUNCK;
        // uint8_t data[_DATA_READ_CHUNCK];
        while (_thread_control)
        {
            ssize_t str_len = _rs485Connection.Read(tram);

            if (str_len != -1)
            {
                for (ssize_t i = 0; i < str_len; i++)
                {
                    _parseQueue.push_back((uint8_t)tram.data.at(i));
                }
                _cvReaderParser.notify_all();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void RS485Provider::writeData()
    {
        std::unique_lock<std::mutex> _lockWriter(_mtxWriter);

        // close the thread.
        while (_thread_control)
        {
            // read until the start there or the queue is empty
            _cvReaderWriter.wait(_lockWriter, [&] { return !_writerQueue.empty(); });

            queueObject msg = _writerQueue.get_n_pop_front();
            const size_t data_size = msg.data.size() + 7;
            uint8_t *data = new uint8_t[data_size];
            data[0] = _START_BYTE;
            data[1] = msg.slave;
            data[2] = msg.cmd;
            data[3] = static_cast<uint8_t>(msg.data.size());

            std::vector<uint8_t> data_vec;
            for (int i = 0; i < data[3]; i++)
            {
                data[i + 4] = msg.data[i];
                data_vec.push_back(msg.data[i]);
            }

            std::tuple<uint8_t, uint8_t> checksum = checkSum(data[1], data[2], data[3], data_vec);

            data[data_size - 3] = std::get<0>(checksum);
            data[data_size - 2] = std::get<1>(checksum);
            data[data_size - 1] = _END_BYTE;
            sonia_common_cpp::SerialTram tram;
            tram.data.assign(data, data + data_size);
            // tram.data = data;
            tram.size = data_size;
            _rs485Connection.Transmit(tram);
            delete data;
        }
    }

    void RS485Provider::parseData()
    {
        std::vector<uint8_t> psu_volt_array[4];
        std::vector<uint8_t> psu_curr_array[4];
        std::vector<uint8_t> psu_feed_array[4];

        std::unique_lock<std::mutex> _lockParser(_mtxParser);

        while (_thread_control)
        {
            // read until the start there or the queue is empty
            _cvReaderParser.wait(_lockParser, [&] { return !_parseQueue.empty(); });
            // check if the bit is the start bit:
            if (_parseQueue.front() != _START_BYTE)
            {
                _parseQueue.pop_front();
            }
            else
            {
                sonia_common_ros2::msg::RS485msg msgRS485 = sonia_common_ros2::msg::RS485msg();

                // pop the unused start data
                _parseQueue.pop_front();

                msgRS485.slave = _parseQueue.get_n_pop_front();
                msgRS485.cmd = _parseQueue.get_n_pop_front();

                uint8_t nbByte = _parseQueue.get_n_pop_front();

                for (int i = 0; i < nbByte; i++)
                {
                    msgRS485.data.push_back(_parseQueue.get_n_pop_front());
                }

                std::tuple<uint8_t, uint8_t> checkResult = {(_parseQueue.get_n_pop_front()),
                                                            _parseQueue.get_n_pop_front()};

                // pop the unused end data
                _parseQueue.pop_front();

                std::tuple<uint8_t, uint8_t> calc_checksum =
                    checkSum(msgRS485.slave, msgRS485.cmd, nbByte, msgRS485.data);
                // if the checksum is bad, drop the packet
                if (checkResult == calc_checksum)
                {
                    // publisher.publish(msg);
                    switch (msgRS485.slave)
                    {
                        case SlaveId::SLAVE_KILLMISSION:
                            _publisherKill->publish(msgRS485);
                            break;
                        case SlaveId::SLAVE_IO:
                            _publisherIO->publish(msgRS485);
                            break;
                        case SlaveId::SLAVE_PWR_MANAGEMENT:
                        case SlaveId::SLAVE_PSU0:
                        case SlaveId::SLAVE_PSU1:
                        case SlaveId::SLAVE_PSU2:
                        case SlaveId::SLAVE_PSU3:
                            _publisherMotor->publish(msgRS485);
                            break;
                        default:
                            RCLCPP_WARN(this->get_logger(), "Unknown slave: %X", msgRS485.slave);
                            break;
                    }
                }
                // packet dropped
            }
        }
    }
}  // namespace rs485_port_manager
