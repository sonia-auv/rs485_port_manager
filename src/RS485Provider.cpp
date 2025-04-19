#include "rs485_port_manager/RS485Provider.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace rs485_port_manager
{

    RS485Provider::RS485Provider()
        : Node("rs485_provider"), _rs485Connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        try
        {
            auv = std::getenv("AUV");
            if (strcmp(auv, "AUV8")==0 || strcmp(auv, "LOCAL")==0)
            {
                ESC_SLAVE = _SlaveId::SLAVE_PWR_MANAGEMENT;
                RCLCPP_INFO(this->get_logger(), "Using AUV8 port");
            }
            else
                if(strcmp(auv, "AUV7")==0)
                {
                    ESC_SLAVE = _SlaveId::SLAVE_ESC;
                    RCLCPP_INFO(this->get_logger(), "Using AUV7 port");
                }
        }
        catch (...)
        {
            ESC_SLAVE = _SlaveId::SLAVE_ESC;
        }

        group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = group1;

        _reader = std::thread(std::bind(&RS485Provider::readData, this));
        _writer = std::thread(std::bind(&RS485Provider::writeData, this));
        _parser = std::thread(std::bind(&RS485Provider::parseData, this));
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", 10);
        _publisherMission =
            this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _publisherMotorVoltages =
            this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_voltages", 10);
        _publisherMotorCurrents =
            this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_currents", 10);
        _publisherMotorTemperature = this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>(
            "/provider_power/motor_temperatures", 10);
        _publisherBatteryVoltages = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_voltages", 10);
        _publisherBatteryCurrents = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_currents", 10);
        _publisherBatteryTemperature = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_temperatures", 10);
        _publisherMotorFeedback =
            this->create_publisher<sonia_common_ros2::msg::MotorFeedback>("/provider_power/motor_feedback", 10);
        _actuatorService = this->create_service<sonia_common_ros2::srv::ActuatorService>(
            "/provider_actuator/do_action", std::bind(&RS485Provider::processActuatorRequest, this, _1, _2));
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&RS485Provider::pollKillMission, this));
        // _timerPowerRequest= this->create_wall_timer(500ms, std::bind(&RS485Provider::pollPower, this));

        _publisherThrusterPwm =
            this->create_publisher<sonia_common_ros2::msg::MotorPwm>("/provider_thruster/thruster_pwm", 10);
        _subscriberThrusterPwm = this->create_subscription<sonia_common_ros2::msg::MotorPwm>(
            "/provider_thruster/thruster_pwm", 10, std::bind(&RS485Provider::PwmCallback, this, _1), sub_opt);
        _subscriberMotorOnOff = this->create_subscription<std_msgs::msg::Bool>(
            "/provider_power/activate_motors", 10, std::bind(&RS485Provider::EnableDisableMotors, this, _1), sub_opt);
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

    void RS485Provider::pollKillMission()
    {
        queueObject msg;
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_KILL;
        _writerQueue.push_back(msg);
        
        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_MISSION;
        _writerQueue.push_back(msg);
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
                response->success = true;
                break;
            }
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_TORPEDO:
            {
                ser.cmd=_Cmd::CMD_IO_TORPEDO_ACTION;
                ser.data.push_back(request->side);
                _writerQueue.push_back(ser);
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

    void RS485Provider::publishMotor(uint8_t cmd, std::vector<float> data)
    {
        /*if (data.size() != 8)
        {
            return;
        }*/
        sonia_common_ros2::msg::MotorPowerMessages msg;
        msg.motor1 = data[0];
        msg.motor2 = data[1];
        msg.motor3 = data[2];
        msg.motor4 = data[3];
        msg.motor5 = data[4];
        msg.motor6 = data[5];
        msg.motor7 = data[6];
        msg.motor8 = data[7];

        switch (cmd)
        {
            case _Cmd::CMD_VOLTAGE:
                _publisherMotorVoltages->publish(msg);
                break;
            case _Cmd::CMD_CURRENT:
                _publisherMotorCurrents->publish(msg);
                break;
            case _Cmd::CMD_TEMPERATURE:
                _publisherMotorTemperature->publish(msg);
                break;
            default:
                break;
        }
    }

    void RS485Provider::publishBattery(uint8_t cmd, float *data)
    {
        sonia_common_ros2::msg::BatteryPowerMessages msg;
        msg.battery1 = data[0];
        msg.battery2 = data[1];

        switch (cmd)
        {
            case _Cmd::CMD_VOLTAGE:
                _publisherBatteryVoltages->publish(msg);
                break;
            case _Cmd::CMD_CURRENT:
                _publisherBatteryCurrents->publish(msg);
                break;
            case _Cmd::CMD_TEMPERATURE:
                _publisherBatteryTemperature->publish(msg);
                break;
            default:
                break;
        }
    }

    void RS485Provider::publishMotorFeedback(std::vector<uint8_t> data)
    {
        sonia_common_ros2::msg::MotorFeedback msg;
        msg.motor1 = data[0];
        msg.motor2 = data[1];
        msg.motor3 = data[2];
        msg.motor4 = data[3];
        msg.motor5 = data[4];
        msg.motor6 = data[5];
        msg.motor7 = data[6];
        msg.motor8 = data[7];
        _publisherMotorFeedback->publish(msg);
    }

    void RS485Provider::processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data)
    {
        std::vector<float> motorData;
        motorData.reserve(10);
        float batteryData[2];

        switch (cmd)
        {
            case _Cmd::CMD_VOLTAGE:

                if (convertBytesToFloat(data, motorData, nb_thruster + nb_battery) < 0)
                {
                    std::cerr << "ERROR in the message. Dropping VOLTAGE packet" << std::endl;
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotor(_Cmd::CMD_VOLTAGE, motorData);
                publishBattery(_Cmd::CMD_VOLTAGE, batteryData);

                break;
            case _Cmd::CMD_CURRENT:

                if (convertBytesToFloat(data, motorData, nb_thruster + nb_battery) < 0)
                {
                    std::cerr << "ERROR in the message. Dropping CURRENT packet" << std::endl;
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotor(_Cmd::CMD_CURRENT, motorData);
                publishBattery(_Cmd::CMD_CURRENT, batteryData);
                break;
            case _Cmd::CMD_TEMPERATURE:

                if (convertBytesToFloat(data, motorData, nb_thruster + nb_battery) < 0)
                {
                    std::cerr << "ERROR in the message. Dropping TEMPERATURE packet" << std::endl;
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotor(_Cmd::CMD_TEMPERATURE, motorData);
                publishBattery(_Cmd::CMD_TEMPERATURE, batteryData);
                break;
            case _Cmd::CMD_READ_MOTOR:                
                publishMotorFeedback(data);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "CMD Not identified");
                break;
        }
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
            }
        }
    }

    void RS485Provider::writeData()
    {
        // close the thread.
        while (_thread_control)
        {            
            // pause the thread.
            while (!_writerQueue.empty())
            {
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
    }

    void RS485Provider::parseData()
    {

        std::vector<uint8_t> psu_array [8];
        
        while (_thread_control)
        {
            // read until the start there or the queue is empty
            while (!_parseQueue.empty())
            {
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
                                if(msg.cmd==_Cmd::CMD_VOLTAGE)
                                    psu_array[0]=msg.data;
                                if(msg.cmd==_Cmd::CMD_CURRENT)
                                    psu_array[4]=msg.data;
                                break;
                            case _SlaveId::SLAVE_PSU1:
                                if(msg.cmd==_Cmd::CMD_VOLTAGE)
                                    psu_array[1]=msg.data;
                                if(msg.cmd==_Cmd::CMD_CURRENT)
                                    psu_array[5]=msg.data;
                                break;
                            case _SlaveId::SLAVE_PSU2:
                                if(msg.cmd==_Cmd::CMD_VOLTAGE)
                                    psu_array[2]=msg.data;
                                if(msg.cmd==_Cmd::CMD_CURRENT)
                                    psu_array[6]=msg.data;
                                break;
                            case _SlaveId::SLAVE_PSU3:
                                if(msg.cmd==_Cmd::CMD_VOLTAGE)
                                    psu_array[3]=msg.data;
                                if(msg.cmd==_Cmd::CMD_CURRENT)
                                    psu_array[7]=msg.data;
                                break;
                            default:
                                RCLCPP_WARN(this->get_logger(), "Unknown slave: %X", msg.slave);
                                break;
                        }
                        if(std::all_of(std::begin(psu_array), std::end(psu_array), [](const std::vector<uint8_t>& psu){return !psu.empty();})){
                            std::vector<uint8_t> pwr_msg;
                            
                            pwr_msg.push_back(psu_array[0][3]);
                            pwr_msg.push_back(psu_array[2][3]);

                            pwr_msg.push_back(psu_array[0][0]);
                            pwr_msg.push_back(psu_array[1][0]);
                            pwr_msg.push_back(psu_array[2][0]);
                            pwr_msg.push_back(psu_array[3][0]);
                            pwr_msg.push_back(psu_array[0][1]);
                            pwr_msg.push_back(psu_array[1][1]);
                            pwr_msg.push_back(psu_array[2][1]);
                            pwr_msg.push_back(psu_array[3][1]);

                            if(pwr_msg.size()==10)
                                processPowerManagement(_Cmd::CMD_VOLTAGE, pwr_msg);

                            pwr_msg.push_back(psu_array[5][3]);
                            pwr_msg.push_back(psu_array[6][3]);

                            pwr_msg.push_back(psu_array[4][0]);
                            pwr_msg.push_back(psu_array[5][0]);
                            pwr_msg.push_back(psu_array[6][0]);
                            pwr_msg.push_back(psu_array[7][0]);
                            pwr_msg.push_back(psu_array[4][1]);
                            pwr_msg.push_back(psu_array[5][1]);
                            pwr_msg.push_back(psu_array[6][1]);
                            pwr_msg.push_back(psu_array[7][1]);

                            if(pwr_msg.size()==10)
                                processPowerManagement(_Cmd::CMD_CURRENT, pwr_msg);

                        }
                        
                    }
                    // packet dropped
                }
            }
        }
    }

    int RS485Provider::convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size)
    {
        uint8_t size_req = req.size();
        if (size_req % 4 != 0) return -1;

        _bytesToFloat converter;

        for (uint8_t i = 0; i < size; ++i)  // shifting of 4 for each data
        {
            converter.bytes[0] = req[4 * i];
            converter.bytes[1] = req[4 * i + 1];
            converter.bytes[2] = req[4 * i + 2];
            converter.bytes[3] = req[4 * i + 3];
            res.push_back(converter.value);
        }
        return 0;
    }
    void RS485Provider::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_ACT_MOTOR;
        switch (ESC_SLAVE)
        {
            // AUV8 motor control
            case _SlaveId::SLAVE_PWR_MANAGEMENT:
                ser.slave = ESC_SLAVE;
                ToggleMotors(msg.data, nb_thruster, ser.data);
                _writerQueue.push_back(ser);
                break;
            // AUV7 motor control
            case _SlaveId::SLAVE_ESC:
                ser.slave = _SlaveId::SLAVE_PSU0;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU1;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU2;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU3;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);
                _writerQueue.push_back(ser);
                break;
            default:
                break;
        }
    }
    void RS485Provider::ToggleMotors(const bool state, const uint8_t size, std::vector<uint8_t> &data)
    {
        if (state)
        {
            for (size_t i = 0; i < size; i++)
            {
                data.push_back(1);
            }
        }
        else
        {
            for (size_t i = 0; i < size; i++)
            {
                data.push_back(0);
            }
        }
    }
    void RS485Provider::PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_PWM;
        switch (ESC_SLAVE)
        {
            case _SlaveId::SLAVE_PWR_MANAGEMENT:
                ser.data.clear();
                ser.slave = ESC_SLAVE;

                ser.data.push_back(msg.motor1 >> 8);
                ser.data.push_back(msg.motor1 & 0xFF);

                ser.data.push_back(msg.motor2 >> 8);
                ser.data.push_back(msg.motor2 & 0xFF);

                ser.data.push_back(msg.motor3 >> 8);
                ser.data.push_back(msg.motor3 & 0xFF);

                ser.data.push_back(msg.motor4 >> 8);
                ser.data.push_back(msg.motor4 & 0xFF);

                ser.data.push_back(msg.motor5 >> 8);
                ser.data.push_back(msg.motor5 & 0xFF);

                ser.data.push_back(msg.motor6 >> 8);
                ser.data.push_back(msg.motor6 & 0xFF);

                ser.data.push_back(msg.motor7 >> 8);
                ser.data.push_back(msg.motor7 & 0xFF);

                ser.data.push_back(msg.motor8 >> 8);
                ser.data.push_back(msg.motor8 & 0xFF);

                _writerQueue.push_back(ser);
                break;
            case _SlaveId::SLAVE_ESC:
                ser.slave = _SlaveId::SLAVE_PSU0;
                ser.data.clear();
                ser.data.push_back(msg.motor1 >> 8);
                ser.data.push_back(msg.motor1 & 0xFF);
                ser.data.push_back(msg.motor5 >> 8);
                ser.data.push_back(msg.motor5 & 0xFF);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU1;
                ser.data.clear();
                ser.data.push_back(msg.motor2 >> 8);
                ser.data.push_back(msg.motor2 & 0xFF);
                ser.data.push_back(msg.motor6 >> 8);
                ser.data.push_back(msg.motor6 & 0xFF);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU2;
                ser.data.clear();
                ser.data.push_back(msg.motor3 >> 8);
                ser.data.push_back(msg.motor3 & 0xFF);
                ser.data.push_back(msg.motor7 >> 8);
                ser.data.push_back(msg.motor7 & 0xFF);
                _writerQueue.push_back(ser);

                ser.slave = _SlaveId::SLAVE_PSU3;
                ser.data.clear();
                ser.data.push_back(msg.motor4 >> 8);
                ser.data.push_back(msg.motor4 & 0xFF);
                ser.data.push_back(msg.motor8 >> 8);
                ser.data.push_back(msg.motor8 & 0xFF);
                _writerQueue.push_back(ser);
                break;

            default:
                break;
        }
    }
}  // namespace sonia_hw_interface
