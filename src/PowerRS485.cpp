#include "rs485_port_manager/PowerRS485.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace rs485_port_manager{

    PowerRS485::PowerRS485() 
    : Node("powerRS485_provider"){
        try
        {
            const char *auv = std::getenv("AUV");
            if (strcmp(auv, "AUV8")==0 || strcmp(auv, "LOCAL")==0 || strcmp(auv, "LITE1")==0)
            {
                esc_slave = SlaveId::SLAVE_PWR_MANAGEMENT;
                RCLCPP_INFO(this->get_logger(), "Using %s port", auv);
            }
            else
                RCLCPP_INFO(this->get_logger(), "Unknown ENV var");
        }
        catch (...)
        {
            esc_slave = SlaveId::SLAVE_PWR_MANAGEMENT;
            RCLCPP_INFO(this->get_logger(), "Error, Using default port");
        }

        rs485 = RS485Provider::GetInstance();
        rs485->AddObservateur(this);

        rclcpp::QoS qosRelible(10);
        qosRelible.reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile).history(rclcpp::HistoryPolicy::KeepLast);


        rclcpp::QoS qosBestEffort(10);
        qosBestEffort.reliability(rclcpp::ReliabilityPolicy::BestEffort).durability(rclcpp::DurabilityPolicy::Volatile).history(rclcpp::HistoryPolicy::KeepLast);

        _publisherMotorVoltages =
            this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_voltages", qosRelible);
        _publisherMotorCurrents =
            this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_currents", qosRelible);
        _publisherMotorTemperature = this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>(
            "/provider_power/motor_temperatures", qosBestEffort);
        _publisherBatteryVoltages = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_voltages", qosBestEffort);
        _publisherBatteryCurrents = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_currents", qosBestEffort);
        _publisherBatteryTemperature = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>(
            "/provider_power/battery_temperatures", qosBestEffort);
        _publisherMotorFeedback =
            this->create_publisher<sonia_common_ros2::msg::MotorFeedback>("/provider_power/motor_feedback", qosRelible);
        
        _subscriberThrusterPwm = this->create_subscription<sonia_common_ros2::msg::MotorPwm>(
            "/provider_thruster/thruster_pwm", qosRelible, std::bind(&PowerRS485::PwmCallback, this, _1));
        _subscriberMotorOnOff = this->create_subscription<std_msgs::msg::Bool>(
            "/provider_power/activate_motors", qosRelible, std::bind(&PowerRS485::EnableDisableMotors, this, _1));
    }

    // node destructor
    PowerRS485::~PowerRS485() {}

    void PowerRS485::sendMessage(queueObject queue){
        rs485->AddMessage(queue);
    }

    void PowerRS485::messageRS485CallBack(queueObject queue){
        processPowerManagement(queue.cmd, queue.data);
        RCLCPP_INFO(this->get_logger(), "motorvolt");    
    }

    void PowerRS485::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        queueObject ser;
        ser.cmd = Cmd::CMD_ACT_MOTOR;
        ser.slave = esc_slave;
        ser.data.reserve(NB_THRUSTER);

        uint8_t value = msg.data ? 1 : 0;

        for (size_t i = 0; i < NB_THRUSTER; ++i)
        {
            ser.data.push_back(value);
        }
        sendMessage(ser);
    }

    void PowerRS485::PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg)
    {
        queueObject ser;
        ser.cmd = Cmd::CMD_PWM;
        ser.slave = esc_slave;
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

        sendMessage(ser);
    }

    void PowerRS485::publishMotorInfo(uint8_t cmd, std::vector<float> data)
    {
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
            case Cmd::CMD_VOLTAGE:
                _publisherMotorVoltages->publish(msg);
                break;
            case Cmd::CMD_CURRENT:
                _publisherMotorCurrents->publish(msg);
                break;
            case Cmd::CMD_TEMPERATURE:
                _publisherMotorTemperature->publish(msg);
                break;
            default:
                break;
        }
    }

    void PowerRS485::publishBattery(uint8_t cmd, float *data)
    {
        sonia_common_ros2::msg::BatteryPowerMessages msg;
        msg.battery1 = data[0];
        msg.battery2 = data[1];

        switch (cmd)
        {
            case Cmd::CMD_VOLTAGE:
                _publisherBatteryVoltages->publish(msg);
                break;
            case Cmd::CMD_CURRENT:
                _publisherBatteryCurrents->publish(msg);
                break;
            case Cmd::CMD_TEMPERATURE:
                _publisherBatteryTemperature->publish(msg);
                break;
            default:
                break;
        }
    }

    void PowerRS485::publishMotorFeedback(std::vector<uint8_t> data)
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

    void PowerRS485::processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data)
    {
        std::vector<float> motorData;
        motorData.reserve(10);
        float batteryData[2];

        switch (cmd)
        {
            case Cmd::CMD_VOLTAGE:

                if (convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
                {
                    RCLCPP_ERROR(this->get_logger(),  "ERROR in the message. Dropping VOLTAGE packet");
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotorInfo(Cmd::CMD_VOLTAGE, motorData);
                publishBattery(Cmd::CMD_VOLTAGE, batteryData);

                break;
            case Cmd::CMD_CURRENT:

                if (convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
                {
                    RCLCPP_ERROR(this->get_logger(),  "ERROR in the message. Dropping CURRENT packet");
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotorInfo(Cmd::CMD_CURRENT, motorData);
                publishBattery(Cmd::CMD_CURRENT, batteryData);
                break;
            case Cmd::CMD_TEMPERATURE:

                if (convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
                {
                    RCLCPP_ERROR(this->get_logger(),  "ERROR in the message. Dropping TEMPERATURE packet");
                    return;
                }

                batteryData[0] = motorData[motorData.size() - 2];
                batteryData[1] = motorData[motorData.size() - 1];
                motorData.pop_back();
                motorData.pop_back();

                publishMotorInfo(Cmd::CMD_TEMPERATURE, motorData);
                publishBattery(Cmd::CMD_TEMPERATURE, batteryData);
                break;
            case Cmd::CMD_READ_MOTOR:                
                publishMotorFeedback(data);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "CMD Not identified");
                break;
        }
    }

    int PowerRS485::convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size)
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
}