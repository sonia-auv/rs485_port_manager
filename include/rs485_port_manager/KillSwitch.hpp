#pragma once

#include <stdio.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <tuple>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "sonia_common_cpp/SerialConn.hpp"
#include "sonia_common_cpp/SharedQueue.hpp"
#include "sonia_common_ros2/msg/kill_status.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"
#include "InterfaceModule.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"
namespace module
{

    class KillProvider : public rclcpp::Node, InterfaceModule
    {
        
        public:
            KillProvider();
            ~KillProvider();

        /**
         * @brief Kill all internal threads.
         */
        void Kill();

        /**
         * @brief
         *
         * @param status
         */
        void sendMessage(queueObject queue);

        /**
         * @brief
         *
         * @param msg
         */
        void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg);

        private:
        /**
         * @brief Polls the status of kill and mission via RS485 connection.
         *
         * This function sends requests to retrieve the status of kill and mission
         * via the RS485 connection.
         */
        void pollKillMission();

        /**
         * @brief
         *
         * @param status
         */
        void publishKill(bool status);

        /**
         * @brief
         *
         * @param status
         */
        void publishMission(bool status);

        /**
         * @brief
         *
         * @param status
         */

        /* Used to publish the information of the Kill Switch */
        rclcpp::Subscription<sonia_common_ros2::msg::RS485msg>::SharedPtr  _subscriberKill;
        rclcpp::Publisher<sonia_common_ros2::msg::KillStatus>::SharedPtr _publisherKill;
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Publisher<sonia_common_ros2::msg::MissionStatus>::SharedPtr _publisherMission;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
        rclcpp::CallbackGroup::SharedPtr group1;
    };

}  // namespace KillManager