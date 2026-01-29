#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "sonia_common_ros2/msg/battery_power_messages.hpp"
#include "sonia_common_ros2/msg/motor_feedback.hpp"
#include "sonia_common_ros2/msg/motor_power_messages.hpp"
#include "sonia_common_ros2/msg/motor_pwm.hpp"

#include "InterfaceModuleRS485.hpp"
#include "RS485Driver.hpp"

namespace rs485_port_manager{

    /**
     * @class PowerRS485
     * @brief Power provider node that collects power information from the thrusters and batteries
     */
    class PowerRS485 : InterfaceModuleRS485, public rclcpp::Node
    {
        public:

            PowerRS485();
            ~PowerRS485() = default;

        private:
            RS485Driver *rs485;

            /**
             * @brief Publishes motor information (voltage, current or temperature).
             * @param cmd Command defining what type of data is sent.
             * @param data Vector of data to publish.
             */
            void publishMotorInfo(uint8_t cmd, std::vector<float> data);

            /**
             * @brief Publishes thruster feedback if enabled or disabled.
             * @param data Vector of data to publish.
             */
            void publishMotorFeedback(const std::vector<uint8_t> data);

            /**
             * @brief Publishes battery information (voltage, current or temperature).
             * @param cmd Command defining what type of data is sent.
             * @param data  The battery data to publish.
             */
            void publishBattery(uint8_t cmd, float *data);

            /**
             * @brief Handles enabling or disabling all thrusters.
             * @param msg Command to enabled or disable the thrusters.
             */
            void EnableDisableMotors(const std_msgs::msg::Bool &msg);

            /**
             * @brief Callback that recieves PWM to send to the thrusters.
             * @param msg Array of PWM of all thrusters.
             */
            void PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg);

            /**
             * @brief Processe power information from the power management in the prototype.
             * @param cmd Command defining what type of data is processed.
             * @param data  Power data to be processed.
             */
            void processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data);

            /**
             * @brief Processe power information from the power management in the prototype.
             * @param req Byte data to be converted.
             * @param res  Float data array to store converted result.
             * @param size Size of the result 
             */
            int convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size);

            /**
             * @brief method to send message to rs485
             *
             * @param queue the queueObject to send to RS485
             */
            void sendMessage(queueObject queue) override;

            /**
             * @brief Method to read message from rs485
             *
             * @param msg the message from RS485
             */
            void messageRS485CallBack(queueObject queue) override;
            
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

            uint8_t esc_slave;

            const uint8_t NB_THRUSTER = 8;
            const uint8_t NB_BATTERY = 2;

            union _bytesToFloat
            {
                uint8_t bytes[4];
                float_t value;
            };
    };
}