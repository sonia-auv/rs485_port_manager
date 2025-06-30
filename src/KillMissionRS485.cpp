#include "rs485_port_manager/KillMissionRS485.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    KillMissionRS485::KillMissionRS485() // Node constructor
    : Node("rs485_kill_switch")
    {
        rclcpp::CallbackGroup::SharedPtr group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = group1;

        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/kill_switch_rs485/kill_status", 10);
        _publishers485 = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/msgToSend", 10);
        _publisherMission =
            this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _subscriberKill = this->create_subscription<sonia_common_ros2::msg::RS485msg>("/rs485/killMessage", 10, 
            std::bind(&KillMissionRS485::messageRS485CallBack, this, _1), sub_opt);
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&KillMissionRS485::pollKillMission, this));
    }
    KillMissionRS485::~KillMissionRS485() {}

    void KillMissionRS485::sendMessage(queueObject queue)
    {
        auto to_return = sonia_common_ros2::msg::RS485msg();
        to_return.slave = queue.slave;
        to_return.cmd = queue.cmd;
        to_return.data = queue.data;

        _publishers485->publish(to_return);
    }
    void KillMissionRS485::pollKillMission()
    {
        queueObject msg;
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_KILL;
        
        sendMessage(msg);

        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_MISSION;

        sendMessage(msg);
    }
    void KillMissionRS485::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg)
    {
        queueObject ser;
        ser.cmd = msg.cmd;
        ser.slave = msg.slave;
        ser.data.clear();

        std::vector<uint8_t> uint8Vector;
        uint8Vector.resize(msg.data.size());

        std::transform(msg.data.begin(), msg.data.end(), uint8Vector.begin(),
                   [](char c) { return static_cast<uint8_t>(c); });
        
        switch (ser.cmd)
        {
            case _Cmd::CMD_KILL:
                // get data value
                // publish on kill publisher
                publishKill(ser.data[0] == 1);
                break;
            case _Cmd::CMD_MISSION:
                // get data value
                // publish on mission publisher
                publishMission(ser.data[0] == 1);
                break;
        }

    }
    void KillMissionRS485::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }
    void KillMissionRS485::publishMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisherMission->publish(state);
    }
}