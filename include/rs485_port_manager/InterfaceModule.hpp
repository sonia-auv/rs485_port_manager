#pragma once

#include <stdio.h>
#include <rclcpp/rclcpp.hpp>

#include "Definition.hpp"

namespace module
{

    class InterfaceModule
    {
        public:

            /**
             * @brief method to send message to rs485
             * 
             * queue : Message to send by RS485
             */
            void sendMessage(queueObject queue){};

            /**
             * @brief Method to read message from rs485
             */
            virtual void receiveMessage(const sonia_common_ros2::msg::RS485msg &msg){};

        private:
    };

}