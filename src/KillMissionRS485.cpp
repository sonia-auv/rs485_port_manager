#include "rs485_port_manager/KillMissionRS485.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    KillMissionRS485::KillMissionRS485() // Node constructor
    : Node("rs485_kill_switch")
    {
        rs485 = RS485Provider::GetInstance();
        rs485->AddObservateur(this);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile).history(rclcpp::HistoryPolicy::KeepLast);

        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", qos);
        _publisherMission =
            this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", qos);
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&KillMissionRS485::pollKillMission, this));
    }

    KillMissionRS485::~KillMissionRS485(){}

    void KillMissionRS485::sendMessage(queueObject queue)
    {
        rs485->AddMessage(queue);
    }
    void KillMissionRS485::pollKillMission()
    {
        queueObject msg;
        // dummy data to create a valid message, 
        // the data was not read when you get the status
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        msg.slave = SlaveId::SLAVE_KILLMISSION;
        msg.cmd = Cmd::CMD_KILL;
        
        sendMessage(msg);

        // Wait for a short duration to allow for processing... Embeded restriction
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Transmit request to get mission status
        msg.slave = SlaveId::SLAVE_KILLMISSION;
        msg.cmd = Cmd::CMD_MISSION;

        sendMessage(msg);
    }
    void KillMissionRS485::messageRS485CallBack(queueObject queue)
    {
        
        if(queue.slave == SlaveId::SLAVE_KILLMISSION){
            switch (queue.cmd)
            {
                case Cmd::CMD_KILL:
                    // get data value
                    // publish on kill publisher
                    publishKill(queue.data[0] == 1);
                    break;
                case Cmd::CMD_MISSION:
                    // get data value
                    // publish on mission publisher
                    publishMission(queue.data[0] == 1);
                    break;
            }
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