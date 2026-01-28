#include "rs485_port_manager/RS485Provider.hpp"

using namespace std::chrono_literals;
namespace rs485_port_manager
{
    RS485Provider::RS485Provider() // Node constructor
    : Node("rs485_provider")
    {        
        _publisherNodeStatus = this->create_publisher<sonia_common_ros2::msg::NodeStatus>("/system_monitor/node_status", 1);

        //wall timer
        _timerNodeStatus = this->create_wall_timer(500ms, std::bind(&RS485Provider::publishStatus, this));

        _node_status.node_name = this->get_name();
        _node_status.quality = sonia_common_ros2::msg::NodeStatus::Q_OK;
        _node_status.state = sonia_common_ros2::msg::NodeStatus::STATE_RUNNING;
    }
   
    void RS485Provider::publishStatus(){
        RCLCPP_INFO(this->get_logger(), "publishing node status: %s", _node_status.node_name.c_str());
        _node_status.stamp = this->now();
        _publisherNodeStatus->publish(_node_status);
    }
} // namespace rs485_port_manager