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

#include "sonia_common_cpp/SharedQueue.hpp"
#include "sonia_common_cpp/SerialConn.hpp"
#include "sonia_common_ros2/msg/kill_status.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"

#include "Definition.hpp"

namespace kill_switch_port_manager
{

    class KillProvider : public rclcpp::Node
    {
        public:

        /**
         * @brief Kill all internal threads.
         */
        void Kill();
        KillProvider();
        ~KillProvider();

        private:
        /**
         * @brief Polls the status of kill and mission via RS485 connection.
         *
         * This function sends requests to retrieve the status of kill and mission
         * via the RS485 connection.
         */
        void pollKillMission();

        /**
         * @brief Open designated internal serial port.
         *
         * @return bool True if opened successfully else False.
         */
        bool OpenPort();


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

        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::KillStatus>::SharedPtr _publisherKill;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
        sonia_common_cpp::SharedQueue<queueObject> _writerQueue;

        sonia_common_cpp::SerialConn _rs485Connection;
        bool _thread_control;
    };

}  // namespace KillManager