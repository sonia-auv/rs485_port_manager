#include "rs485_port_manager/ArmControlLogic.hpp"
#include "rs485_port_manager/MotorRS485.hpp"


using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace arm_control_logic
{

    std::vector<uint8_t> arm_control_logic::ArmControlLogic::motorsProcessing(const int16_t _motor1, const int16_t _motor2)
    {
        std::vector<uint8_t> data_motors;
        uint16_t value_motor1;
        uint16_t value_motor2;
        if ((_motor1<=limite_max_motor1 && _motor1>=limite_min_motor1) && (_motor2<=limite_max_motor2 && _motor2>=limite_min_motor2)) {
            data_motors.push_back(CMD_MOTOR);
            value_motor1=_motor1;
            value_motor2=_motor2;
        }
        else if ((_motor1<=limite_max_motor1 && _motor1>=limite_min_motor1) && (_motor2>limite_max_motor2)){
            data_motors.push_back(CMD_MOTOR);
            value_motor1=_motor1;
            value_motor2=limite_max_motor2;
        }
        else if ((_motor1<=limite_max_motor1 && _motor1>=limite_min_motor1) && (_motor2<limite_min_motor2)){
            data_motors.push_back(CMD_MOTOR);
            value_motor1=_motor1;
            value_motor2=limite_min_motor2;
        }
        else if ((_motor1>limite_max_motor1) && (_motor2<=limite_max_motor2 && _motor2>=limite_min_motor2)){
            data_motors.push_back(CMD_MOTOR);
            value_motor1=limite_max_motor1;
            value_motor2=_motor2;
        }
        else if ((_motor1<limite_min_motor1) && (_motor2<=limite_max_motor2 && _motor2>=limite_min_motor2)){
            data_motors.push_back(CMD_MOTOR);
            value_motor1=limite_min_motor1;
            value_motor2=_motor2;
        }
        data_motors.push_back((uint8_t)((value_motor1 >> 8) & 0xff));
        data_motors.push_back((uint8_t)((value_motor1 >> 0) & 0xff));
        data_motors.push_back((uint8_t)((value_motor2 >> 8) & 0xff));
        data_motors.push_back((uint8_t)((value_motor2 >> 0) & 0xff));

        return data_motors;
    }

    std::vector<uint8_t> arm_control_logic::ArmControlLogic::staticPosProcessing(const uint16_t _static_pos_wanted, const uint16_t current_motor1, const uint16_t current_motor2)
    {
        std::vector<uint8_t> data_static_pos;
        uint16_t value_motor1;
        uint16_t value_motor2;
        switch (_static_pos_wanted){
            case 0:
                //value_motor1= home_motor1;///////////////////////////////////////////////////////////////////////////////////////////def dans un fichier config les home_motor et repos_motor
                //value_motor2= home_motor2;

                break;
            case 1:
                //value_motor1= repos_motor1;
                //value_motor2= repos_motor2;

                break;
            default :
                value_motor1= current_motor1;
                value_motor2= current_motor2;
                break;
        }
        data_static_pos=this->motorsProcessing(value_motor1,value_motor2);

        return data_static_pos;
    }

    std::vector<uint8_t> arm_control_logic::ArmControlLogic::grabberProcessing(const float _value)
    {
        std::vector<uint8_t> data_grabber;
        rs485_port_manager::MotorRS485::_bytesToFloat converter;
        if (limite_min_grabber<=_value && _value<=limite_max_grabber){
            data_grabber.push_back(CMD_GRABBER);
            converter.value=_value;
        }
        else if (limite_min_grabber>_value){
            data_grabber.push_back(CMD_OUT);
            converter.value=limite_min_grabber;
        }
        else{
            data_grabber.push_back(CMD_OUT);
            converter.value=limite_max_grabber;
        }
        data_grabber.push_back(converter.bytes[0]);
        data_grabber.push_back(converter.bytes[1]);
        data_grabber.push_back(converter.bytes[2]);
        data_grabber.push_back(converter.bytes[3]);

        return data_grabber;
    }

}


