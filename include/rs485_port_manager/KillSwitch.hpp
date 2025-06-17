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
#include "sonia_common_ros2/msg/kill_status.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"

    /**
     * @brief Internal Queue Object
     *
     */
    struct queueObject
    {
        uint8_t slave;
        uint8_t cmd;
        std::vector<uint8_t> data;
        void printTram()
        {
            printf("%x ", slave);
            printf("%x ", cmd);
            for (size_t i = 0; i < data.size(); i++)
            {
                printf("%x ", data[i]);
            }
            
            printf("\n");
        }
    };

    class KillProvider : public rclcpp::Node
    {
        public:

        /**
         * @brief Kill all internal threads.
         */
        void Kill();

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

        /* Used to publish the information of the Kill Switch */
        rclcpp::Publisher<sonia_common_ros2::msg::KillStatus>::SharedPtr _publisherKill;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
    };

}  // namespace KillManager