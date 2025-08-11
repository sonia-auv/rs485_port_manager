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
            

        private:


            // Boundaries for motors values to avoid hitting the sub
            uint16_t min_boundary_motor1=500;//////////////////////////////////////////////à modifier si nécessaire
            uint16_t max_boundary_motor1=2500;//////////////////////////////////////////////à modifier si nécessaire
            uint16_t min_boundary_motor2=500;//////////////////////////////////////////////à modifier si nécessaire
            uint16_t max_boundary_motor2=2500;//////////////////////////////////////////////à modifier si nécessaire
            

            // Boundaries for grabber values
            float min_boundary_grabber=1;//////////////////////////////////////////////à modifier si nécessaire
            float max_boundary_grabber=100;//////////////////////////////////////////////à modifier si nécessaire
        
    };

}  // namespace