#include "rs485_port_manager/LedRS485.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    LedRS485::LedRS485(): Node("rs485_com")
    {
        _publisherLed = this->create_publisher<std_msgs::msg::Bool>("/provider_rs485/led_status", 10);

        _timerLedState= this->create_wall_timer(500ms, std::bind(&LedRS485::pollLedState, this));

    }
    void LedRS485::pollLedState()
    {
        queueObject msg;
        // dummy data to create a valid message, 
        // the data was not read when you get the status
        msg.data.push_back(0x00);

        // Transmit request to get kill status
        //msg.slave = 0;
        //msg.cmd = 0;
        
        sendMessage(msg);

    }

    void LedRS485::sendMessage(queueObject queue)
    {
        rs485->AddMessage(queue);
    }

    void LedRS485::messageRS485CallBack(queueObject queue)
    {

    }
}