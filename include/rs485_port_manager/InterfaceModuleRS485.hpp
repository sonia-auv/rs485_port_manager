#pragma once

#include <stdio.h>

#include <rclcpp/rclcpp.hpp>

#include "CommonDefinitionRS485.hpp"
#include "sonia_common_ros2/msg/rs485msg.hpp"

namespace rs485_port_manager
{

    class InterfaceModuleRS485
    {
        public:
        /**
         * @brief method to send message to rs485
         *
         * queue : Message to send by RS485
         */
        virtual void sendMessage(queueObject queue) = 0;

        /**
         * @brief Method to read message from rs485
         */
        virtual void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg) = 0;
    };

}  // namespace rs485_port_manager