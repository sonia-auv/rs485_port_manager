#include "rs485_port_manager/KillSwitch.hpp"

using namespace std::chrono_literals;

namespace module
{
    KillProvider::KillProvider() // Node constructor
    : Node("rs485_kill_switch")
    {
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/kill_switch_rs485/kill_status", 10);
        _publishers485 = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/msgToSend", 10);
        _publisherMission =
            this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&ĶillProvider::pollKillMission, this));
    }
    KillProvider::~KillProvider() {}

    void KillProvider::sendMessage(queueObject queue)
    {
        auto to_return = sonia_common_ros2::msg::RS485msg();
        to_return.slave = queue.slave;
        to_return.cmd = queue.cmd;
        to_return.data = queue.data;

        _publishers485->publish(to_return);
    }
    void KillProvider::pollKillMission()
    {
        queueObject msg;
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_KILL;
        
        // sendMessage(msg);

        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_MISSION;

        // sendMessage(msg);
    }
    void KillProvider::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg)
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
    void KillProvider::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }
    void KillProvider::publishMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisherMission->publish(state);
    }
}