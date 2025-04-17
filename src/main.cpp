#include "rs485_port_manager/RS485Provider.hpp"
#include <stdlib.h>

#include <chrono>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rs485 = std::make_shared<rs485_port_manager::RS485Provider>();
    if (!rs485->OpenPort())
    {
        printf("Could not open port...\n");
        return EXIT_FAILURE;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rs485);
    executor.spin();
    // rclcpp::spin(rs485);

    rclcpp::shutdown();

    rs485->Kill();
    return EXIT_SUCCESS;
}
