#include "rs485_port_manager/MotorRS485.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace rs485_port_manager{


    MotorRS485::MotorRS485() 
    : Node("motorRS485_provider"){
        try
        {
            const char *auv = std::getenv("AUV");
            if (strcmp(auv, "AUV8")==0 || strcmp(auv, "LOCAL")==0)
            {
                esc_slave = SlaveId::SLAVE_PWR_MANAGEMENT;
                RCLCPP_INFO(this->get_logger(), "Using AUV8 port");
            }
            else if(strcmp(auv, "AUV7")==0)
            {
                esc_slave = SlaveId::SLAVE_ESC;
                RCLCPP_INFO(this->get_logger(), "Using AUV7 port");
            }
            else 
            {
                esc_slave = SlaveId::SLAVE_ESC;
                RCLCPP_INFO(this->get_logger(), "Using default port AUV7");
            }

        }
        catch (...)
        {
            esc_slave = SlaveId::SLAVE_ESC;
            RCLCPP_INFO(this->get_logger(), "Error, Using default port AUV7");
        }

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
            "/provider_thruster/thruster_pwm", 10, std::bind(&MotorRS485::PwmCallback, this, _1));
        _subscriberMotorOnOff = this->create_subscription<std_msgs::msg::Bool>(
            "/provider_power/activate_motors", 10, std::bind(&MotorRS485::EnableDisableMotors, this, _1));
        
        _publisherRS485 = this->create_publisher<sonia_common_ros2::msg::RS485msg>(
            "/rs485/msgToSend", 10);

        _subscriberMotor = this->create_subscription<sonia_common_ros2::msg::RS485msg>("/rs485/motorMessage", 10
                , std::bind(&MotorRS485::messageRS485CallBack, this, _1));
    }

    // node destructor
    MotorRS485::~MotorRS485() {}

    void MotorRS485::sendMessage(queueObject queue){
        sonia_common_ros2::msg::RS485msg msgRS485 = sonia_common_ros2::msg::RS485msg();

        msgRS485.cmd = queue.cmd;
        msgRS485.slave = queue.slave;
        msgRS485.data = queue.data;

        _publisherRS485->publish(msgRS485);
    }

    void MotorRS485::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg){

        queueObject ser;
        ser.cmd = msg.cmd;
        ser.slave = msg.slave;

        ser.data = msg.data;
        switch (ser.slave)
        {
            case SlaveId::SLAVE_PWR_MANAGEMENT:
                processPowerManagement(ser.cmd, ser.data);
                break;
            case SlaveId::SLAVE_PSU0:
            {
                //Motor 1 and Motor 5
                switch (ser.cmd)
                {
                    case Cmd::CMD_VOLTAGE:
                        psu_volt_array[0]=ser.data;
                        break;
                    case Cmd::CMD_CURRENT:
                        psu_curr_array[0]=ser.data;
                        break;
                    case Cmd::CMD_READ_MOTOR:
                        psu_feed_array[0]=ser.data;
                        break;                                        
                    default:
                        break;
                }//end switch case
                break;
            }
            case SlaveId::SLAVE_PSU1:
            {
                //Motor 2 and Motor 6
                switch (ser.cmd)
                {
                    case Cmd::CMD_VOLTAGE: psu_volt_array[1]=ser.data;
                        break;
                    case Cmd::CMD_CURRENT: psu_curr_array[1]=ser.data;
                        break;
                    case Cmd::CMD_READ_MOTOR:  psu_feed_array[1]=ser.data;
                        break;                                        
                    default:
                        break;
                }//end switch case
                break;
            }
            case SlaveId::SLAVE_PSU2:
            {
                //Motor 3 and Motor 7
                switch (msg.cmd)
                {
                    case Cmd::CMD_VOLTAGE: psu_volt_array[2]=ser.data;
                        break;
                    case Cmd::CMD_CURRENT: psu_curr_array[2]=ser.data;
                        break;
                    case Cmd::CMD_READ_MOTOR:  psu_feed_array[2]=ser.data;
                        break;                                        
                    default:
                        break;
                }//end switch case
                break;
            }
            case SlaveId::SLAVE_PSU3:
            {
                //Motor 4 and Motor 8
                switch (msg.cmd)
                {
                    case Cmd::CMD_VOLTAGE: psu_volt_array[3]=ser.data;
                        break;
                    case Cmd::CMD_CURRENT: psu_curr_array[3]=ser.data;
                        break;
                    case Cmd::CMD_READ_MOTOR:  psu_feed_array[3]=ser.data;
                        break;                                        
                    default:
                        break;
                }//end switch case  
                break;
            }
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown slave: %X", ser.slave);
                break;
        }
        
        if(ser.slave==SlaveId::SLAVE_PSU0 || ser.slave==SlaveId::SLAVE_PSU1 || ser.slave==SlaveId::SLAVE_PSU2 || ser.slave==SlaveId::SLAVE_PSU3 ){
                switch(ser.cmd){
                    case Cmd::CMD_VOLTAGE:
                    {
                        processAUV7PowerManagement(Cmd::CMD_VOLTAGE, psu_volt_array);
                        break;
                    }
                    case Cmd::CMD_CURRENT:
                    {
                        processAUV7PowerManagement(Cmd::CMD_CURRENT, psu_curr_array); 
                        break;
                    }
                    case Cmd::CMD_READ_MOTOR:
                    {
                        processAUV7PowerManagement(Cmd::CMD_READ_MOTOR, psu_feed_array); 
                        break;
                    }
                    default:{
                        RCLCPP_ERROR(this->get_logger(), "ERROR, Unkown CMD for AUV7 pwr management");
                    }
                }
            }
}

    void MotorRS485::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        queueObject ser;
        ser.cmd = Cmd::CMD_ACT_MOTOR;
        switch (esc_slave)
        {
            // AUV8 motor control
            case SlaveId::SLAVE_PWR_MANAGEMENT:
                ser.slave = esc_slave;
                ToggleMotors(msg.data, NB_THRUSTER, ser.data);

                sendMessage(ser);
                break;
            // AUV7 motor control
            case SlaveId::SLAVE_ESC:
                ser.slave = SlaveId::SLAVE_PSU0;
                ser.data.clear();
                ToggleMotors(msg.data, NB_THRUSTER_BY_PSU_AUV7, ser.data);

                sendMessage(ser);

                ser.slave = SlaveId::SLAVE_PSU1;
                ser.data.clear();
                ToggleMotors(msg.data, NB_THRUSTER_BY_PSU_AUV7, ser.data);

                sendMessage(ser);

                ser.slave = SlaveId::SLAVE_PSU2;
                ser.data.clear();
                ToggleMotors(msg.data, NB_THRUSTER_BY_PSU_AUV7, ser.data);

                sendMessage(ser);

                ser.slave = SlaveId::SLAVE_PSU3;
                ser.data.clear();
                ToggleMotors(msg.data, NB_THRUSTER_BY_PSU_AUV7, ser.data);

                sendMessage(ser);
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

    void MotorRS485::publishMotorInfo(uint8_t cmd, std::vector<float> data)
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

    void MotorRS485::publishBattery(uint8_t cmd, float *data)
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

    void MotorRS485::processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data)
    {
        std::vector<float> motorData;
        motorData.reserve(10);
        float batteryData[2];

        switch (cmd)
        {
            case Cmd::CMD_VOLTAGE:

                if (RS485Utils::convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
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

                if (RS485Utils::convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
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

                if (RS485Utils::convertBytesToFloat(data, motorData, NB_THRUSTER + NB_BATTERY) < 0)
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

    void MotorRS485::processAUV7PowerManagement(const uint8_t cmd, std::vector<uint8_t> (&psu_data)[4]){
        std::vector<float> motorData;
        motorData.reserve(8);
        float batteryData[2];
        
        if(checkNoEmptyVector(psu_data)){
            switch(cmd){
                case Cmd::CMD_VOLTAGE:
                {
                    std::vector<float> convertData[4];
                    for(size_t i=0; i<4; i++){    
                        convertData[i].reserve(psu_data[i].size()/4);
                        if (RS485Utils::convertBytesToFloat(psu_data[i], convertData[i], psu_data[i].size()/4)<0)
                        {
                            RCLCPP_ERROR(this->get_logger(),  "ERROR in the message. Dropping VOLTAGE packet");
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
                    publishMotorInfo(Cmd::CMD_VOLTAGE, motorData);
                    publishBattery(Cmd::CMD_VOLTAGE, batteryData);
                    break;
                }
                case Cmd::CMD_CURRENT:
                {
                    std::vector<float> convertData[4];
                    for(size_t i=0; i<4; i++){    
                        convertData[i].reserve(psu_data[i].size()/4);
                        if (RS485Utils::convertBytesToFloat(psu_data[i], convertData[i], psu_data[i].size()/4)<0)
                        {
                            RCLCPP_ERROR(this->get_logger(),  "ERROR in the message. Dropping CURRENT packet");
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
                    batteryData[0]=(convertData[0].at(2)+convertData[1].at(2))/2;
                    batteryData[1]=(convertData[2].at(2)+convertData[3].at(2))/2;
                    //publish currents
                    publishMotorInfo(Cmd::CMD_CURRENT, motorData);
                    publishBattery(Cmd::CMD_CURRENT, batteryData);
                    break;
                }
                case Cmd::CMD_READ_MOTOR:
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

}