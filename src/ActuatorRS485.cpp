#include "rs485_port_manager/ActuatorRS485.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    ActuatorRS485::ActuatorRS485() // Node constructor
    : Node("rs485_io_module")
    {
        rs485 = RS485Provider::GetInstance();
        rs485->AddObservateur(this);
        _actuatorService = this->create_service<sonia_common_ros2::srv::ActuatorService>(
            "/provider_actuator/do_action", std::bind(&ActuatorRS485::processActuatorRequest, this, _1, _2));
    }
    ActuatorRS485::~ActuatorRS485() {}

    void ActuatorRS485::processActuatorRequest(
    const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
    std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response)
    {
        // Variables for transmission status and data vector
        queueObject ser;
        ser.slave=SlaveId::SLAVE_IO;
        
        switch (request->element){
            case  sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_DROPPER:
            {
                ser.cmd=Cmd::CMD_IO_DROPPER_ACTION;
                ser.data.push_back(request->side);

                sendMessage(ser);
                response->success = true;
                break;
            }
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_TORPEDO:
            {
                ser.cmd=Cmd::CMD_IO_TORPEDO_ACTION;
                ser.data.push_back(request->side);

                sendMessage(ser);
                response->success = true;
                break;
            }
            default:
                std::cerr << "ERROR in element. Unknown element" << std::endl;
                response->success = false;
                break;
        }         
    }
    void ActuatorRS485::sendMessage(queueObject queue)
    {
        rs485->AddMessage(queue);
    }

    void ActuatorRS485::messageRS485CallBack(queueObject queue)
    {

    }
}