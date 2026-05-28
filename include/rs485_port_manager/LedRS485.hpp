#pragma once

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

#include "sonia_common_ros2/msg/rs485msg.hpp"
#include "InterfaceModuleRS485.hpp"
#include "RS485Driver.hpp"

namespace rs485_port_manager
{
    class LedRS485 : InterfaceModuleRS485, public rclcpp::Node
    {
        public:
            LedRS485();
            ~LedRS485() = default;

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
        private:
            RS485Driver *rs485;

            void toggleLeds(const std_msgs::msg::Bool msg);

            rclcpp::TimerBase::SharedPtr _timerLedState;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriptionLed;

    };
} //namespace rs485_port_manager