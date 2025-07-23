#include "rs485_port_manager/ActuatorRS485.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    ActuatorRS485::ActuatorRS485()  // Node constructor
        : Node("rs485_io_module")
    {
        _publishers485 = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/msgToSend", 10);
        _actuatorService = this->create_service<sonia_common_ros2::srv::ActuatorService>(
            "/provider_actuator/do_action", std::bind(&ActuatorRS485::processActuatorRequest, this, _1, _2));
    }

    void ActuatorRS485::processActuatorRequest(
        const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
        std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response)
    {
        // Variables for transmission status and data vector
        queueObject ser;
        ser.slave = SlaveId::SLAVE_IO;

        switch (request->element)
        {
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_DROPPER:
            {
                ser.cmd = Cmd::CMD_IO_DROPPER_ACTION;
                ser.data.push_back(request->side);

                sendMessage(ser);
                response->success = true;
                break;
            }
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_TORPEDO:
            {
                ser.cmd = Cmd::CMD_IO_TORPEDO_ACTION;
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
        sonia_common_ros2::msg::RS485msg to_return = sonia_common_ros2::msg::RS485msg();
        to_return.slave = queue.slave;
        to_return.cmd = queue.cmd;
        to_return.data = queue.data;

        _publishers485->publish(to_return);
    }

    void ActuatorRS485::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg) { (void)msg; }
}  // namespace rs485_port_manager