#include "rs485_port_manager/LedRS485.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    LedRS485::LedRS485(): Node("provider_com")
    {
        _subscriptionLed = this->create_subscription<std_msgs::msg::Bool>("/provider_com/enable_led", 1, std::bind(&LedRS485::toggleLeds, this, _1));
        rs485 = RS485Driver::GetInstance();
    }
    void LedRS485::toggleLeds(const std_msgs::msg::Bool msg)
    {
        queueObject led;

        led.slave = SlaveId::SLAVE_LED;

        if(msg.data)
        { //green color for testing

            for(size_t i = 0; i<30; i++){
                led.data.push_back(170);
                led.data.push_back(255);
                led.data.push_back(0);
            }   
        }
        else
        {
            for(size_t i = 0; i<30; i++){
                led.data.push_back(0);
                led.data.push_back(0);
                led.data.push_back(0);
            } 
        }
        
        sendMessage(led);
    }

    void LedRS485::sendMessage(queueObject queue)
    {
        rs485->AddMessage(queue);
    }

    void LedRS485::messageRS485CallBack(queueObject queue)
    {
        
    }
}