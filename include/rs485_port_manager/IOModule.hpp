#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"

#include "sonia_common_ros2/srv/actuator_service.hpp"
#include "sonia_common_ros2/msg/rs485msg.hpp"
#include "InterfaceModuleRS485.hpp"
#include "sonia_common_ros2/msg/motors_values_arm.hpp"
#include "sonia_common_ros2/srv/static_pos.hpp"

using _MotorsValuesArm = sonia_common_ros2::msg::MotorsValuesArm;
using _StaticPos=sonia_common_ros2::srv::StaticPos;

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

        void armMotorsCallback(const _MotorsValuesArm::SharedPtr msg);

        void armStaticPos(const std::shared_ptr<_StaticPos::Request> request, std::shared_ptr<_StaticPos::Response> response);


        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Service<sonia_common_ros2::srv::ActuatorService>::SharedPtr _actuatorService;
        
        // Subscriber reading the three motor values
        rclcpp::Subscription<_MotorsValuesArm>::SharedPtr armMotorsSubscriber;

        // Service to move robot arm to specific static positions (home)
        rclcpp::Service<_StaticPos>::SharedPtr armStaticPosSrv;

        
    };

}  // namespace KillManager