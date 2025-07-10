#pragma once

#include <stdio.h>

#include <functional>
#include <std_msgs/msg/bool.hpp>
#include <tuple>
#include <vector>


namespace arm_control_logic
{

    class ArmControlLogic: public
    {
        
        public:
            ArmControlLogic();
            ~ArmControlLogic();
        

        private:

            std::vector<uint8_t> motorsProcessing (const int16 _motor1, const int16 _motor2);

            std::vector<uint8_t> staticPosProcessing(const uint16 _static_pos_wanted);

            std::vector<uint8_t> grabberProcessing(const float _value);
        
    };

}  // namespace