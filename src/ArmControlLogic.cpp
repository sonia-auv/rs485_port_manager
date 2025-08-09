#include "rs485_port_manager/ArmControlLogic.hpp"


// Class computing basic functions processing motors values, static position and grabber for robotic arm


namespace rs485_port_manager
{

    ArmControlLogic::ArmControlLogic(){}
    ArmControlLogic::~ArmControlLogic(){}

    // Basic function used to create a uint8_t vector, including both motors values and cmd which is reused to set up the queue object sent in IO Module
    std::vector<uint8_t> ArmControlLogic::motorsProcessing(const uint16_t _motor1, const uint16_t _motor2)
    {
        std::vector<uint8_t> data_motors;
        uint16_t value_motor1;
        uint16_t value_motor2;


        //////////////////////////////////// DOESN'T WORK BECAUSE MOTOR1 BOUNDARIES DEPEND OF MOTOR2 BOUNDARIES
        ////////////////////////////////// IF MOTOR1 =90 DEGREES THEN MOTOR2 DOESN'T HAVE ANY BOUNDARY
        ////////////////////////////////// IF MOTOR1 = 0 DEGREES THEN MOTOR2 HAVE TO BE BETWEEN 180 AND 360
        //////////////////////////// MORE INTRERESTING TO DO THE BOUNDARY TEST IN ARMCALCULATOR.CPP
        
        // Assign _motorX value, if inside the boundaries, to value_motorX, the boundary exceeded value otherwise
        value_motor1=std::clamp(_motor1,min_boundary_motor1,max_boundary_motor1);
        value_motor2=std::clamp(_motor2,min_boundary_motor2,max_boundary_motor2);



        // Necessary if we decide to send CMD_OUT insted of CMD_MOTOR when values are outside the boundaries 
        if ((value_motor1==max_boundary_motor1 or value_motor1==min_boundary_motor1) or (value_motor2==max_boundary_motor2 or value_motor2==min_boundary_motor2))
            data_motors.push_back(Cmd::CMD_MOTOR);
        else
            data_motors.push_back(Cmd::CMD_MOTOR);

        data_motors.push_back((uint8_t)((value_motor1 >> 8) & 0xff));
        data_motors.push_back((uint8_t)((value_motor1 >> 0) & 0xff));
        data_motors.push_back((uint8_t)((value_motor2 >> 8) & 0xff));
        data_motors.push_back((uint8_t)((value_motor2 >> 0) & 0xff));

        return data_motors;
    }


    // Basic function used to create a uint8_t vector, including both motors values and cmd(according to _static_pos_wanted) which is reused to set up the queue object sent in IO Module
    std::vector<uint8_t> ArmControlLogic::staticPosProcessing(const uint8_t _static_pos_wanted, const uint16_t current_motor1, const uint16_t current_motor2)
    {
        std::vector<uint8_t> data_static_pos;
        uint16_t value_motor1;
        uint16_t value_motor2;
        switch (_static_pos_wanted){
            case 0:
                value_motor1= 700;//home_motor1;///////////////////////////////////////////////////////////////////////////////////////////def dans un fichier config les home_motor et repos_motor
                value_motor2= 700;//home_motor2;

                break;
            case 1:
                value_motor1= 700;//repos_motor1;
                value_motor2= 500;//repos_motor2;

                break;
            default :
                value_motor1= current_motor1;
                value_motor2= current_motor2;
                break;
        }
        data_static_pos=this->motorsProcessing(value_motor1,value_motor2);

        return data_static_pos;
    }

    // Basic function used to create a uint8_t vector, including grabber value and cmd which is reused to set up the queue object sent in IO Module
    std::vector<uint8_t> ArmControlLogic::grabberProcessing(const float _value)
    {
        std::vector<uint8_t> data_grabber;
        rs485_port_manager::bytesToFloat converter;
        data_grabber.push_back(Cmd::CMD_GRABBER);
        converter.value=std::clamp(_value,min_boundary_grabber,max_boundary_grabber);


        data_grabber.push_back(converter.bytes[0]);
        data_grabber.push_back(converter.bytes[1]);
        data_grabber.push_back(converter.bytes[2]);
        data_grabber.push_back(converter.bytes[3]);

        return data_grabber;
    }

}


