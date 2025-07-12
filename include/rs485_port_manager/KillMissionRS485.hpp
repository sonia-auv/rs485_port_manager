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
#include "InterfaceModuleRS485.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"

namespace rs485_port_manager
{

    class KillMissionRS485 : public rclcpp::Node, InterfaceModuleRS485
    {
        
        public:
            KillMissionRS485();
            ~KillMissionRS485();

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

        private:
        /**
         * @brief Polls the status of kill and mission via RS485 connection.
         *
         * This function sends requests to retrieve the status of kill and mission
         * via the RS485 connection.
         */
        void pollKillMission();

        /**
         * @brief function to publish on topic for status of kill switch
         *
         * @param status
         */
        void publishKill(bool status);

        /**
         * @brief function to publish on topic for status of mission switch
         *
         * @param status
         */
        void publishMission(bool status);

        /* Used to publish the information of the Kill Switch */
        rclcpp::Subscription<sonia_common_ros2::msg::RS485msg>::SharedPtr  _subscriberKill;
        rclcpp::Publisher<sonia_common_ros2::msg::KillStatus>::SharedPtr _publisherKill;
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publishers485;
        rclcpp::Publisher<sonia_common_ros2::msg::MissionStatus>::SharedPtr _publisherMission;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
    };

}  // namespace KillManager