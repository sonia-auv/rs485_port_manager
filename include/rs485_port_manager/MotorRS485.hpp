#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "sonia_common_ros2/msg/battery_power_messages.hpp"
#include "sonia_common_ros2/msg/motor_feedback.hpp"
#include "sonia_common_ros2/msg/motor_power_messages.hpp"
#include "sonia_common_ros2/msg/motor_pwm.hpp"

#include "InterfaceModule.hpp"

namespace module{

    class MotorRS485 : InterfaceModule, public rclcpp::Node
    {
        public:

            MotorRS485();
            ~MotorRS485();

        private:

        std::vector<uint8_t> psu_volt_array[4];
        std::vector<uint8_t> psu_curr_array[4];
        std::vector<uint8_t> psu_feed_array[4];

        const uint8_t nb_thruster = 8;
        const uint8_t nb_battery = 2;

        void publishMotor(uint8_t cmd, std::vector<float> data);

        void publishBattery(uint8_t cmd, float *data);

        void publishMotorFeedback(std::vector<uint8_t> data);

        void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg);

        void EnableDisableMotors(const std_msgs::msg::Bool &msg);
        void ToggleMotors(const bool state, const uint8_t size, std::vector<uint8_t> &data);
        void PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg);

        void processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data);
        void processAUV7PowerManagement(const uint8_t cmd, std::vector<uint8_t> (&psu_data)[4]);

        bool checkNoEmptyVector(std::vector<uint8_t> (&array)[4]);

        rclcpp::CallbackGroup::SharedPtr group1;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorVoltages;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryVoltages;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorCurrents;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryCurrents;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorTemperature;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryTemperature;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorFeedback>::SharedPtr _publisherMotorFeedback;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPwm>::SharedPtr _publisherThrusterPwm;
        rclcpp::Subscription<sonia_common_ros2::msg::MotorPwm>::SharedPtr _subscriberThrusterPwm;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriberMotorOnOff;

        rclcpp::Subscription<sonia_common_ros2::msg::RS485msg>::SharedPtr _subscriberMotor;
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publisherRS485;

        const char *auv;

        uint8_t ESC_SLAVE;

        union _bytesToFloat
        {
            uint8_t bytes[4];
            float_t value;
        };

        int convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size);
    };
}