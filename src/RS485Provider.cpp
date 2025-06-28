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

        _reader = std::thread(std::bind(&RS485Provider::readData, this));
        _writer = std::thread(std::bind(&RS485Provider::writeData, this));
        _parser = std::thread(std::bind(&RS485Provider::parseData, this));
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", 10);
        _publisherMission =
            this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _actuatorService = this->create_service<sonia_common_ros2::srv::ActuatorService>(
            "/provider_actuator/do_action", std::bind(&RS485Provider::processActuatorRequest, this, _1, _2));
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&RS485Provider::pollKillMission, this));
        // _timerPowerRequest= this->create_wall_timer(500ms, std::bind(&RS485Provider::pollPower, this));
        
        _publisherRS485 = this->create_subscription<sonia_common_ros2::msg::RS485msg>(
            "/rs485/msgToSend", 10, std::bind(&RS485Provider::RS485callback, this, _1), sub_opt);
    }

    // node destructor
    RS485Provider::~RS485Provider() {}

    bool RS485Provider::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    void RS485Provider::RS485callback(const sonia_common_ros2::msg::RS485msg &msg)
    {
        queueObject ser;
        ser.cmd = msg.cmd;
        ser.slave = msg.slave;
        ser.data.clear();

        std::vector<uint8_t> uint8Vector;
        uint8Vector.resize(msg.data.size());
        
        std::transform(msg.data.begin(), msg.data.end(), uint8Vector.begin(),
                   [](char c) { return static_cast<uint8_t>(c); });
        ser.data = uint8Vector;
        
        _writerQueue.push_back(ser);
        _cvReaderWriter.notify_all();
    }

    void RS485Provider::pollKillMission()
    {
        queueObject msg;
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_KILL;
        _writerQueue.push_back(msg);
        _cvReaderWriter.notify_all();
        
        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_MISSION;
        _writerQueue.push_back(msg);
        _cvReaderWriter.notify_all();
    }

    void RS485Provider::pollPower()
    {
        _rs485Connection.Transmit(_GET_POWER_MSG, 15);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        _rs485Connection.Transmit(_GET_FEEDBACK_MSG, 15);
    }

    std::tuple<uint8_t, uint8_t> RS485Provider::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte,
                                                          std::vector<uint8_t> data)
    {
        uint16_t check = (uint16_t)(_START_BYTE + slave + cmd + nbByte + _END_BYTE);
        for (uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }
        return {check >> 8, check & 0XFF};
    }

    void RS485Provider::Kill() { _thread_control = false; }

    void RS485Provider::processActuatorRequest(
        const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
        std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response)
    {
        // Variables for transmission status and data vector
        queueObject ser;
        ser.slave=_SlaveId::SLAVE_IO;
        
        switch (request->element){
            case  sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_DROPPER:
            {
                ser.cmd=_Cmd::CMD_IO_DROPPER_ACTION;
                ser.data.push_back(request->side);
                _writerQueue.push_back(ser);
                _cvReaderWriter.notify_all();
                response->success = true;
                break;
            }
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_TORPEDO:
            {
                ser.cmd=_Cmd::CMD_IO_TORPEDO_ACTION;
                ser.data.push_back(request->side);
                _writerQueue.push_back(ser);
                _cvReaderWriter.notify_all();
                response->success = true;
                break;
            }
            default:
                std::cerr << "ERROR in element. Unknown element" << std::endl;
                response->success = false;
                break;
        }         
    }

    void RS485Provider::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }

    void RS485Provider::publishMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisherMission->publish(state);
    }

    void RS485Provider::readData()
    {
        // Delay for port opening
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        uint8_t data[_DATA_READ_CHUNCK];
        while (_thread_control)
        {
            ssize_t str_len = _rs485Connection.ReadPackets(_DATA_READ_CHUNCK, data);

            if (str_len != -1)
            {
                for (ssize_t i = 0; i < str_len; i++)
                {
                    _parseQueue.push_back((uint8_t)data[i]);
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
            _cvReaderWriter.wait(_lockWriter, [&]{ return !_writerQueue.empty(); });

            queueObject msg = _writerQueue.get_n_pop_front();
            const size_t data_size = msg.data.size() + 7;
            uint8_t *data = new uint8_t[data_size];
            data[0] = _START_BYTE;
            data[1] = msg.slave;
            data[2] = msg.cmd;
            data[3] = (uint8_t)msg.data.size();

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
            _rs485Connection.Transmit(data, data_size);
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
        _cvReaderParser.wait(_lockParser, [&]{ return !_parseQueue.empty(); });
            // check if the bit is the start bit:
            if (_parseQueue.front() != _START_BYTE)
            {
                _parseQueue.pop_front();
            }
            else
            {
                queueObject msg;

                // pop the unused start data
                _parseQueue.pop_front();

                msg.slave = _parseQueue.get_n_pop_front();
                msg.cmd = _parseQueue.get_n_pop_front();
                uint8_t nbByte = _parseQueue.get_n_pop_front();

                for (int i = 0; i < nbByte; i++)
                {
                    msg.data.push_back(_parseQueue.get_n_pop_front());
                }

                std::tuple<uint8_t, uint8_t> checkResult = {(_parseQueue.get_n_pop_front()),
                                                            _parseQueue.get_n_pop_front()};

                // pop the unused end data
                _parseQueue.pop_front();

                std::tuple<uint8_t, uint8_t> calc_checksum = checkSum(msg.slave, msg.cmd, nbByte, msg.data);
                // if the checksum is bad, drop the packet
                if (checkResult == calc_checksum)
                {
                    // publisher.publish(msg);
                    switch (msg.slave)
                    {
                        case _SlaveId::SLAVE_KILLMISSION:
                            switch (msg.cmd)
                            {
                                case _Cmd::CMD_KILL:
                                    // get data value
                                    // publish on kill publisher
                                    publishKill(msg.data[0] == 1);
                                    break;
                                case _Cmd::CMD_MISSION:
                                    // get data value
                                    // publish on mission publisher
                                    publishMission(msg.data[0] == 1);
                                    break;
                                default:
                                    break;
                            }
                            break;
                        case _SlaveId::SLAVE_PWR_MANAGEMENT:
                            processPowerManagement(msg.cmd, msg.data);
                            break;
                        case _SlaveId::SLAVE_IO:
                            break;
                        case _SlaveId::SLAVE_PSU0:
                        {
                            //Motor 1 and Motor 5
                            switch (msg.cmd)
                            {
                                case _Cmd::CMD_VOLTAGE:
                                    psu_volt_array[0]=msg.data;
                                    break;
                                case _Cmd::CMD_CURRENT:
                                    psu_curr_array[0]=msg.data;
                                    break;
                                case _Cmd::CMD_READ_MOTOR:
                                    psu_feed_array[0]=msg.data;
                                    break;                                        
                                default:
                                    break;
                            }//end switch case
                            break;
                        }
                        case _SlaveId::SLAVE_PSU1:
                        {
                            //Motor 2 and Motor 6
                            switch (msg.cmd)
                            {
                                case _Cmd::CMD_VOLTAGE: psu_volt_array[1]=msg.data;
                                    break;
                                case _Cmd::CMD_CURRENT: psu_curr_array[1]=msg.data;
                                    break;
                                case _Cmd::CMD_READ_MOTOR:  psu_feed_array[1]=msg.data;
                                    break;                                        
                                default:
                                    break;
                            }//end switch case
                            break;
                        }
                        case _SlaveId::SLAVE_PSU2:
                        {
                            //Motor 3 and Motor 7
                            switch (msg.cmd)
                            {
                                case _Cmd::CMD_VOLTAGE: psu_volt_array[2]=msg.data;
                                    break;
                                case _Cmd::CMD_CURRENT: psu_curr_array[2]=msg.data;
                                    break;
                                case _Cmd::CMD_READ_MOTOR:  psu_feed_array[2]=msg.data;
                                    break;                                        
                                default:
                                    break;
                            }//end switch case
                            break;
                        }
                        case _SlaveId::SLAVE_PSU3:
                        {
                            //Motor 4 and Motor 8
                            switch (msg.cmd)
                            {
                                case _Cmd::CMD_VOLTAGE: psu_volt_array[3]=msg.data;
                                    break;
                                case _Cmd::CMD_CURRENT: psu_curr_array[3]=msg.data;
                                    break;
                                case _Cmd::CMD_READ_MOTOR:  psu_feed_array[3]=msg.data;
                                    break;                                        
                                default:
                                    break;
                            }//end switch case
                            break;
                        }
                        default:
                            RCLCPP_WARN(this->get_logger(), "Unknown slave: %X", msg.slave);
                            break;
                    }
                    
                if(msg.slave==_SlaveId::SLAVE_PSU0 || msg.slave==_SlaveId::SLAVE_PSU1 || msg.slave==_SlaveId::SLAVE_PSU2 || msg.slave==_SlaveId::SLAVE_PSU3 ){
                        switch(msg.cmd){
                            case _Cmd::CMD_VOLTAGE:
                            {
                                processAUV7PowerManagement(_Cmd::CMD_VOLTAGE, psu_volt_array);
                                break;
                            }
                            case _Cmd::CMD_CURRENT:
                            {
                                processAUV7PowerManagement(_Cmd::CMD_CURRENT, psu_curr_array); 
                                break;
                            }
                            case _Cmd::CMD_READ_MOTOR:
                            {
                                processAUV7PowerManagement(_Cmd::CMD_READ_MOTOR, psu_feed_array); 
                                break;
                            }
                            default:{
                                RCLCPP_ERROR(this->get_logger(), "ERROR, Unkown CMD for AUV7 pwr management");
                            }
                        }
                    }
                    
                }
                // packet dropped
            }
        }
    }
}  // namespace rs485_port_manager
