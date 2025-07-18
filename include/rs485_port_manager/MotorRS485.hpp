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
#include "CommonDefinitionRS485.hpp"


namespace rs485_port_manager{

    class MotorRS485 : InterfaceModuleRS485, public rclcpp::Node
    {
        public:

            MotorRS485();
            ~MotorRS485();

        private:

        void publishMotorInfo(uint8_t cmd, std::vector<float> data);

        void publishMotorFeedback(std::vector<uint8_t> data);

        void publishBattery(uint8_t cmd, float *data);

        void EnableDisableMotors(const std_msgs::msg::Bool &msg);

        void ToggleMotors(const bool state, const uint8_t size, std::vector<uint8_t> &data);

        void PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg);

        void processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data);

        void processAUV7PowerManagement(const uint8_t cmd, std::vector<uint8_t> (&psu_data)[4]);

        inline bool checkNoEmptyVector(std::vector<uint8_t> (&array)[4]){
            for (const auto& v: array){
                if(v.empty()){
                    RCLCPP_ERROR(this->get_logger(),  "ERROR in the PSU messages. Dropping PSU messages");
                    return false;
                }
            }
            return true;
        };

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
        void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg) override;
        
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

        uint8_t esc_slave;

        std::vector<uint8_t> psu_volt_array[4];
        std::vector<uint8_t> psu_curr_array[4];
        std::vector<uint8_t> psu_feed_array[4];

        const uint8_t NB_THRUSTER = 8;
        //Two motor by bord
        const uint8_t NB_THRUSTER_BY_PSU_AUV7 = NB_THRUSTER/4;
        const uint8_t NB_BATTERY = 2;

        
    };
}