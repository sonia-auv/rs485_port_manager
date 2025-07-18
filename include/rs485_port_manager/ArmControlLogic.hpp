#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <tuple>
#include <vector>
#include "CommonDefinitionRS485.hpp"


namespace rs485_port_manager
{

    class ArmControlLogic
    {
        
        public:
            ArmControlLogic();
            ~ArmControlLogic();

            std::vector<uint8_t> motorsProcessing (const uint16_t _motor1, const uint16_t _motor2);

            std::vector<uint8_t> staticPosProcessing(const uint8_t _static_pos_wanted, const uint16_t current_motor1, const uint16_t current_motor2);

            std::vector<uint8_t> grabberProcessing(const float _value);

            
        
            uint8_t SLAVE_ARM=9;
            uint8_t CMD_HOME=0;
            uint8_t CMD_REPOS=1;
            uint8_t CMD_MOTOR=2;
            uint8_t CMD_GRABBER=3;
            uint8_t CMD_OUT=255;

        private:


            // Boundaries for motors values
            uint16_t limite_min_motor1=500;
            uint16_t limite_max_motor1=2500;
            uint16_t limite_min_motor2=500;
            uint16_t limite_max_motor2=2500;
            

            // Boundaries for grabber values
            float limite_min_grabber=10;
            float limite_max_grabber=100;
            float error_min_grab =1;
        
    };

}  // namespace