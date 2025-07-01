#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>

#include "sonia_common_ros2/srv/actuator_service.hpp"
#include "sonia_common_ros2/msg/rs485msg.hpp"
#include "InterfaceModuleRS485.hpp"

namespace rs485_port_manager
{

    class IOModule : InterfaceModuleRS485, public rclcpp::Node
    {
        
        public:
            IOModule();
            ~IOModule();
        

        private:

        /**
         * @brief Processes a actuator service request.
         *
         * @param request message to actiavte a IO and data to Send
         * @param response if the message was correctly send by RS485
         */
        void processActuatorRequest(const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
                                   std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response);

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

        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Service<sonia_common_ros2::srv::ActuatorService>::SharedPtr _actuatorService;
    };

}  // namespace KillManager