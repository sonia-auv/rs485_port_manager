#include "rs485_port_manager/IOModule.hpp"
#include "rs485_port_manager/ArmControlLogic.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace rs485_port_manager
{
    IOModule::IOModule() // Node constructor
    : Node("rs485_io_module")
    {

        _publishers485 = this->create_publisher<sonia_common_ros2::msg::RS485msg>("/rs485/msgToSend", 10);
        _actuatorService = this->create_service<sonia_common_ros2::srv::ActuatorService>(
            "/provider_actuator/do_action", std::bind(&IOModule::processActuatorRequest, this, _1, _2));
        
        armMotorsSubscriber = this->create_subscription<_MotorsValuesArm>("/provider_arm/mov", 100, std::bind(&IOModule::armMotorsCallback, this, _1));

        armStaticPosSrv = this->create_service<_StaticPos>("/provider_arm/staticpos", std::bind(&IOModule::armStaticPos, this, _1, _2));

        }
    IOModule::~IOModule() {}

    void IOModule::processActuatorRequest(
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
            case sonia_common_ros2::srv::ActuatorService::Request::ELEMENT_GRABBER:{

                ser.cmd = Cmd::CMD_IO_ARM_GRABBER;
                ser.data.push_back(request->action);
                sendMessage(ser);
                
                response->success=true;
                RCLCPP_INFO(this->get_logger(), "Move to %d position finished ? : yes",request->action);
                break;
            }
            default:
                std::cerr << "ERROR in element. Unknown element" << std::endl;
                response->success = false;
                break;
        }      
        RCLCPP_INFO(this->get_logger(), "Slave : %d",ser.slave);

    }
    void IOModule::sendMessage(queueObject queue)
    {
        sonia_common_ros2::msg::RS485msg to_return = sonia_common_ros2::msg::RS485msg();
        to_return.slave = queue.slave;
        to_return.cmd = queue.cmd;
        to_return.data = queue.data;

        _publishers485->publish(to_return);
    }

    void IOModule::messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg)
    {

    }

    void IOModule::armMotorsCallback(const _MotorsValuesArm::SharedPtr msg) {

        rs485_port_manager::queueObject msgData;
        std::vector<uint8_t> data;

        rs485_port_manager::ArmControlLogic armControlLogic=rs485_port_manager::ArmControlLogic();
        data=armControlLogic.motorsProcessing(msg->motor1 ,msg->motor2);
        msgData.slave = SlaveId::SLAVE_IO;
        msgData.cmd = data[0];
        data.erase(data.begin());
        size_t data_size=data.size();

        for (size_t i = 0; i < data_size; i++){
            msgData.data.push_back(data[i]);
        }
        this->sendMessage(msgData);
        int16_t real_data1;
        int16_t real_data2;
        real_data1=static_cast<unsigned>(msgData.data[0]) << 8 | static_cast<unsigned>(msgData.data[1]);
        real_data2=static_cast<unsigned>(msgData.data[2]) << 8 | static_cast<unsigned>(msgData.data[3]);
        std::cout << "real_data1=" << real_data1 << std::endl;
        std::cout << "real_data2=" << real_data2 << std::endl;
        std::cout << "Send message motors" << std::endl;


    }

    void IOModule::armStaticPos(const std::shared_ptr<_StaticPos::Request> request, std::shared_ptr<_StaticPos::Response> response){
        RCLCPP_INFO(this->get_logger(), "Incoming request\nstatic position is : %d",request->static_pos_wanted);
        std::string static_pos;
        queueObject msg;
        msg.slave = SlaveId::SLAVE_IO;

        switch (request->static_pos_wanted){
            case 0:
                static_pos="home";
                break;
            case 1:
                static_pos="repos";
                break;
            default :
                static_pos="Nowhere";
                RCLCPP_INFO(this->get_logger(), "Wrong value for static position");
                break;
        }
        std::vector<uint8_t> data;

        rs485_port_manager::ArmControlLogic armControlLogic=rs485_port_manager::ArmControlLogic();
        data=armControlLogic.staticPosProcessing(request->static_pos_wanted,1000 ,1000); //define a function to get back the current values of motor1 and motor2
        msg.slave = SlaveId::SLAVE_IO;
        msg.cmd = data[0];
        data.erase(data.begin());
        size_t data_size=data.size();

        for (size_t i = 0; i < data_size; i++){
            msg.data.push_back(data[i]);
        }
        this->sendMessage(msg);
        int16_t real_data1;
        int16_t real_data2;
        real_data1=static_cast<unsigned>(msg.data[0]) << 8 | static_cast<unsigned>(msg.data[1]);
        real_data2=static_cast<unsigned>(msg.data[2]) << 8 | static_cast<unsigned>(msg.data[3]);
        std::cout << "real_data1=" << real_data1 << std::endl;
        std::cout << "real_data2=" << real_data2 << std::endl;
        std::cout << "Send message motors" << std::endl;
    
        response->success=true;
        RCLCPP_INFO(this->get_logger(), "Move to %s finished ? : yes",static_pos.c_str());

    }

}

