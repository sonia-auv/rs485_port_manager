#include "rs485_port_manager/RS485Driver.hpp"
#include "rs485_port_manager/RS485Provider.hpp"
#include "rs485_port_manager/ActuatorRS485.hpp"
#include "rs485_port_manager/KillMissionRS485.hpp"
#include "rs485_port_manager/PowerRS485.hpp"
#include "rs485_port_manager/LedRS485.hpp"

#include <stdlib.h>

#include <chrono>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rs485 = rs485_port_manager::RS485Driver::GetInstance();
    auto providerRS485 = std::make_shared<rs485_port_manager::RS485Provider>();
    auto killRS485 = std::make_shared<rs485_port_manager::KillMissionRS485>();
    auto powerRS485 = std::make_shared<rs485_port_manager::PowerRS485>();
    auto ledRS485 = std::make_shared<rs485_port_manager::LedRS485>();
    auto ioRS485 = std::make_shared<rs485_port_manager::ActuatorRS485>();

    if (!rs485->OpenPort())
    {
        rs485->Kill();
        RCLCPP_FATAL(rclcpp::get_logger("RS485"), "Could not open port...");
        rclcpp::shutdown();
        
        return EXIT_FAILURE;
    }

    rs485->Start();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(killRS485);
    executor.add_node(ledRS485);
    executor.add_node(providerRS485);
    executor.add_node(powerRS485);
    executor.add_node(ioRS485);
    executor.spin();

    rs485->Kill();
    return EXIT_SUCCESS;
}
