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
#include "InterfaceModule.hpp"

namespace module
{

    class IOModule : public rclcpp::Node, InterfaceModule
    {
        
        public:
            IOModule();
            ~IOModule();
        

        private:

        /**
         * @brief Processes a actuator service request.
         *
         * @param request
         * @param response
         */
        void processActuatorRequest(const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
                                   std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response);

        /**
         * @brief
         *
         * @param status
         */
        void sendMessage(queueObject queue) override;

        /**
         * @brief
         *
         * @param msg
         */
        void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg);

        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Service<sonia_common_ros2::srv::ActuatorService>::SharedPtr _actuatorService;
    };

}  // namespace KillManager