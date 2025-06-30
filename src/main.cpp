#include "rs485_port_manager/RS485Provider.hpp"
#include "rs485_port_manager/IOModule.hpp"
#include "rs485_port_manager/KillMissionRS485.hpp"
#include "rs485_port_manager/MotorRS485.hpp"

#include <stdlib.h>

#include <chrono>
#include <iostream>
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rs485 = std::make_shared<rs485_port_manager::RS485Provider>();
    auto killRS485 = std::make_shared<rs485_port_manager::KillMissionRS485>();
    auto motorRS485 = std::make_shared<rs485_port_manager::MotorRS485>();
    auto ioRS485 = std::make_shared<rs485_port_manager::IOModule>();
    if (!rs485->OpenPort())
    {
        printf("Could not open port...\n");
        return EXIT_FAILURE;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rs485);
    executor.add_node(killRS485);
    executor.add_node(motorRS485);
    executor.add_node(ioRS485);
    executor.spin();
    // rclcpp::spin(rs485);

    rclcpp::shutdown();

    rs485->Kill();
    return EXIT_SUCCESS;
}
