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
#include "sonia_common_ros2/msg/motors_values.hpp"
#include "sonia_common_ros2/srv/static_pos.hpp"
#include "sonia_common_ros2/action/grabber.hpp"

using _MotorsValues = sonia_common_ros2::msg::MotorsValues;
using _StaticPos=sonia_common_ros2::srv::StaticPos;
using _Grabber=sonia_common_ros2::action::Grabber;
using _GoalHandleGrabber = rclcpp_action::ServerGoalHandle<_Grabber>;

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

        void armMotorsCallback(const _MotorsValues::SharedPtr msg);

        void armStaticPos(const std::shared_ptr<_StaticPos::Request> request, std::shared_ptr<_StaticPos::Response> response);

        rclcpp_action::GoalResponse grabberGoalHandle(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const _Grabber::Goal> goal);
        rclcpp_action::CancelResponse grabberCancelHandle(const std::shared_ptr<_GoalHandleGrabber> goal_handle);
        void grabberAcceptedHandle(const std::shared_ptr<_GoalHandleGrabber> goal_handle);
        void grabberCallback(const std::shared_ptr<_GoalHandleGrabber> goal_handle);
        void grabberSendValue(const float value);

        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Service<sonia_common_ros2::srv::ActuatorService>::SharedPtr _actuatorService;
        
        // Subscriber reading the three motor values
        rclcpp::Subscription<_MotorsValues>::SharedPtr armMotorsSubscriber;

        // Service to move robot arm to specific static positions (home)
        rclcpp::Service<_StaticPos>::SharedPtr armStaticPosSrv;

        // Action moving grabber
        rclcpp_action::Server<_Grabber>::SharedPtr armGrabberAction;
        std::shared_ptr<_Grabber::Feedback> feedback;
        std::shared_ptr<_Grabber::Result> result;
        
    };

}  // namespace KillManager