#pragma once

#include <stdio.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <tuple>
#include <mutex>
#include <vector>

#include "sonia_common_ros2/msg/node_status.hpp"

namespace rs485_port_manager
{
    class RS485Provider : public rclcpp::Node
    {
        public:
            /**
             * @brief function to get the instance of RS485Provider
             */
            RS485Provider();
            ~RS485Provider() = default;

        private:
            /**
             * @brief Publishes node information of its state and quality.
             */
            void publishStatus();

            rclcpp::Publisher<sonia_common_ros2::msg::NodeStatus>::SharedPtr _publisherNodeStatus;
            
            rclcpp::TimerBase::SharedPtr _timerNodeStatus;

            sonia_common_ros2::msg::NodeStatus _node_status;
    }; 
} //namespace rs485_port_manager