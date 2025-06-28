#include "rs485_port_manager/KillSwitch.hpp"

namespace kill_switch_port_manager
{
    KillProvider::KillProvider()
    : Node("rs485_provider"), _rs485Connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", 10);
    }

    KillProvider::~KillProvider() {}

    // node destructor
    bool KillProvider::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    void KillProvider::pollKillMission()
    {
        queueObject msg;
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_KILL;
        _writerQueue.push_back(msg);
        
        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = _SlaveId::SLAVE_KILLMISSION;
        msg.cmd = _Cmd::CMD_MISSION;
        _writerQueue.push_back(msg);
    }

    void KillProvider::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }
}