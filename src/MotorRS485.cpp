#include "rs485_port_manager/MotorRS485.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace module{


    MotorRS485::MotorRS485() 
    : Node("motorRS485_provider"){
        try
        {
            auv = std::getenv("AUV");
            if (strcmp(auv, "AUV8")==0 || strcmp(auv, "LOCAL")==0)
            {
                ESC_SLAVE = _SlaveId::SLAVE_PWR_MANAGEMENT;
                RCLCPP_INFO(this->get_logger(), "Using AUV8 port");
            }
            else if(strcmp(auv, "AUV7")==0)
            {
                ESC_SLAVE = _SlaveId::SLAVE_ESC;
                RCLCPP_INFO(this->get_logger(), "Using AUV7 port");
            }
            else 
            {
                ESC_SLAVE = _SlaveId::SLAVE_ESC;
                RCLCPP_INFO(this->get_logger(), "Using default port AUV7");
            }

        }
        catch (...)
        {
            ESC_SLAVE = _SlaveId::SLAVE_ESC;
        }

        group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = group1;

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

        _publisherThrusterPwm =
            this->create_publisher<sonia_common_ros2::msg::MotorPwm>("/provider_thruster/thruster_pwm", 10);
        _subscriberThrusterPwm = this->create_subscription<sonia_common_ros2::msg::MotorPwm>(
            "/provider_thruster/thruster_pwm", 10, std::bind(&MotorRS485::PwmCallback, this, _1), sub_opt);
        _subscriberMotorOnOff = this->create_subscription<std_msgs::msg::Bool>(
            "/provider_power/activate_motors", 10, std::bind(&MotorRS485::EnableDisableMotors, this, _1), sub_opt);
    }

    // node destructor
    MotorRS485::~MotorRS485() {}

    void MotorRS485::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_ACT_MOTOR;
        switch (ESC_SLAVE)
        {
            // AUV8 motor control
            case _SlaveId::SLAVE_PWR_MANAGEMENT:
                ser.slave = ESC_SLAVE;
                ToggleMotors(msg.data, nb_thruster, ser.data);
                break;
            // AUV7 motor control
            case _SlaveId::SLAVE_ESC:
                ser.slave = _SlaveId::SLAVE_PSU0;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);

                ser.slave = _SlaveId::SLAVE_PSU1;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);

                ser.slave = _SlaveId::SLAVE_PSU2;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);

                ser.slave = _SlaveId::SLAVE_PSU3;
                ser.data.clear();
                ToggleMotors(msg.data, nb_thruster / 4, ser.data);
                break;
            default:
                break;
        }
    }

    void MotorRS485::ToggleMotors(const bool state, const uint8_t size, std::vector<uint8_t> &data)
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

    void MotorRS485::PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_PWM;
        ser.slave = ESC_SLAVE;
        ser.data.clear();
        
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
        
    }

    void MotorRS485::publishMotor(uint8_t cmd, std::vector<float> data)
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

    void MotorRS485::publishBattery(uint8_t cmd, float *data)
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

    void MotorRS485::publishMotorFeedback(std::vector<uint8_t> data)
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

    bool MotorRS485::checkNoEmptyVector(std::vector<uint8_t> (&array)[4]){
        for (const auto& v: array){
            if(v.empty())return false;
        }
        return true;
    }

    void MotorRS485::processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data)
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

    void MotorRS485::processAUV7PowerManagement(const uint8_t cmd, std::vector<uint8_t> (&psu_data)[4]){
        std::vector<float> motorData;
        motorData.reserve(8);
        float batteryData[2];

        if(checkNoEmptyVector(psu_data)){
            switch(cmd){
                case _Cmd::CMD_VOLTAGE:
                {
                    std::vector<float> convertData[4];
                    for(size_t i=0; i<4; i++){    
                        convertData[i].reserve(psu_data[i].size()/4);
                        if (convertBytesToFloat(psu_data[i], convertData[i], psu_data[i].size()/4)<0)
                        {
                            std::cerr << "ERROR in the message. Dropping VOLTAGE packet" << std::endl;
                            return;
                        }
                    }
                    //8 motor data
                    motorData.push_back(convertData[0].at(0));
                    motorData.push_back(convertData[1].at(0));
                    motorData.push_back(convertData[2].at(0));
                    motorData.push_back(convertData[3].at(0));
                    motorData.push_back(convertData[0].at(1));
                    motorData.push_back(convertData[1].at(1));
                    motorData.push_back(convertData[2].at(1));
                    motorData.push_back(convertData[3].at(1));
            
                    //2 batteries data
                    batteryData[0]=(convertData[0].at(3)+convertData[1].at(3))/2;
                    batteryData[1]=(convertData[2].at(3)+convertData[3].at(3))/2;
                    //publish voltages
                    publishMotor(_Cmd::CMD_VOLTAGE, motorData);
                    publishBattery(_Cmd::CMD_VOLTAGE, batteryData);
                    break;
                }
                case _Cmd::CMD_CURRENT:
                {
                    std::vector<float> convertData[4];
                    for(size_t i=0; i<4; i++){    
                        convertData[i].reserve(psu_data[i].size()/4);
                        if (convertBytesToFloat(psu_data[i], convertData[i], psu_data[i].size()/4)<0)
                        {
                            std::cerr << "ERROR in the message. Dropping CURRENT packet" << std::endl;
                            return;
                        }
                        //std::cerr << "CONVERTDATA: "<<convertData[i].size() << std::endl;
                    }
                    //8 motor data
                    motorData.push_back(convertData[0].at(0));
                    motorData.push_back(convertData[1].at(0));
                    motorData.push_back(convertData[2].at(0));
                    motorData.push_back(convertData[3].at(0));
                    motorData.push_back(convertData[0].at(1));
                    motorData.push_back(convertData[1].at(1));
                    motorData.push_back(convertData[2].at(1));
                    motorData.push_back(convertData[3].at(1));
            
                    //2 batteries data
                    batteryData[0]=(convertData[0].at(2)+convertData[1].at(2))/2;
                    batteryData[1]=(convertData[2].at(2)+convertData[3].at(2))/2;
                    //publish currents
                    publishMotor(_Cmd::CMD_CURRENT, motorData);
                    publishBattery(_Cmd::CMD_CURRENT, batteryData);
                    break;
                }
                case _Cmd::CMD_READ_MOTOR:
                {
                    std::vector<uint8_t> motor_feedback;
                    motor_feedback.push_back(psu_data[0].at(0));
                    motor_feedback.push_back(psu_data[1].at(1));
                    motor_feedback.push_back(psu_data[2].at(0));
                    motor_feedback.push_back(psu_data[3].at(1));
                    motor_feedback.push_back(psu_data[0].at(0));
                    motor_feedback.push_back(psu_data[1].at(1));
                    motor_feedback.push_back(psu_data[2].at(0));
                    motor_feedback.push_back(psu_data[3].at(1));
                    //motor feedback publish
                    publishMotorFeedback(motor_feedback);
                    break;
                }
                default:{
                    RCLCPP_ERROR(this->get_logger(), "ERROR Unkown CMD for PWR management");
                }
            } 
        } 
    }

    int MotorRS485::convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size)
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

    int sendMessage(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size){
        return 1;
    };

    int convertsFloatToByte(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size){
        return 1;
    }
}