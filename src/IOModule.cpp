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
        
        armMotorsSubscriber = this->create_subscription<_MotorsValues>("/provider_arm/mov", 100, std::bind(&IOModule::armMotorsCallback, this, _1));

        armStaticPosSrv = this->create_service<_StaticPos>("/provider_arm/staticpos", std::bind(&IOModule::armStaticPos, this, _1, _2));

        armGrabberAction = rclcpp_action::create_server<_Grabber>(this,"/provider_arm/grabber", std::bind(&IOModule::grabberGoalHandle, this, _1, _2),
        std::bind(&IOModule::grabberCancelHandle, this, _1),
        std::bind(&IOModule::grabberAcceptedHandle, this, _1));
        

        feedback = std::make_shared<_Grabber::Feedback>();
        result = std::make_shared<_Grabber::Result>();
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
            default:
                std::cerr << "ERROR in element. Unknown element" << std::endl;
                response->success = false;
                break;
        }         
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

    void IOModule::armMotorsCallback(const _MotorsValues::SharedPtr msg) {

        rs485_port_manager::queueObject msgData;
        std::vector<uint8_t> data;

        rs485_port_manager::ArmControlLogic armControlLogic=rs485_port_manager::ArmControlLogic();
        data=armControlLogic.motorsProcessing(msg->motor1 ,msg->motor2);
        msgData.slave = SlaveId::SLAVE_ARM;
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
        msg.slave = SLAVE_ARM;

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
        msg.slave = SlaveId::SLAVE_ARM;
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

    rclcpp_action::GoalResponse IOModule::grabberGoalHandle(const rclcpp_action::GoalUUID &uuid,
                                                            std::shared_ptr<const _Grabber::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %f", goal->goal_grabber);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;    
    }

    rclcpp_action::CancelResponse IOModule::grabberCancelHandle(const std::shared_ptr<_GoalHandleGrabber> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void rs485_port_manager::IOModule::grabberAcceptedHandle(const std::shared_ptr<_GoalHandleGrabber> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&IOModule::grabberCallback, this, _1), goal_handle}.detach();
    }


    void rs485_port_manager::IOModule::grabberCallback(const std::shared_ptr<_GoalHandleGrabber> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);// ici faire gaffe au rate
        rclcpp::Rate delay(1/5);// ici faire gaffe au rate
        const auto goal = goal_handle->get_goal();
        int cmp=0;

        this->grabberSendValue(goal->goal_grabber);

        while (cmp<5 && rclcpp::ok()) { //while ((this->feedback->current_width < goal->goal_grabber-error_min_grab && this->feedback->current_width < goal->goal_grabber+error_min_grab)  && rclcpp::ok()) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                this->result->final_width = this->feedback->current_width;
                this->result->success=false;
                goal_handle->canceled(this->result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // Update sequence
            // Publish feedback
            
            cmp++; //ce cmp est juste là pour faire un délai de 5 tour de boucle (qui correspond à 5 secondes si loop_rate =1) pour reproduire l'attente du retour de la pince disant qu'elle a fini d'atteindre la position voulue
            //////////////////////////// IL FAUDRA DONC CHANGER CELA ET LA CONDITION DE LA BOUCLE WHILE QUAND ON POURRA OBTENIR LE RETOUR DE LA PINCE /////////////////////////////////////////////////////////////

            goal_handle->publish_feedback(this->feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");            
            loop_rate.sleep();
        }
        // Check if goal is done
        if (rclcpp::ok()) {
            this->result->final_width = this->feedback->current_width;
            this->result->success=true;
            goal_handle->succeed(this->result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void IOModule::grabberSendValue(const float value) {
        queueObject msg;
        msg.slave = SlaveId::SLAVE_ARM;

        std::vector<uint8_t> data;
        rs485_port_manager::ArmControlLogic armControlLogic=rs485_port_manager::ArmControlLogic();
        data=armControlLogic.grabberProcessing(value);
        msg.cmd = data[0];
        data.erase(data.begin());
        size_t data_size=data.size();

        for (size_t i = 0; i < data_size; i++){
            msg.data.push_back(data[i]);
        }
        this->sendMessage(msg);
        
        std::vector<float> grabber_values;
        rs485_port_manager::RS485Utils::convertBytesToFloat(msg.data,grabber_values,1);
        std::cout << "grabber_values[0]=" << grabber_values[0] << std::endl;
        std::cout << "Send message motors" << std::endl;

    }
}

