#include "hardware_interface_manager/RS485Interface.hpp"
// #include "hardware_interface_manager/ThrusterProvider.hpp"
#include <stdlib.h>

#include <chrono>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto thrust = std::make_shared<sonia_hw_interface::RS485Interface>();
    if (!thrust->OpenPort())
    {
        printf("Could not open port...\n");
        return EXIT_FAILURE;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(thrust);
    executor.spin();
    // rclcpp::spin(thrust);

    rclcpp::shutdown();

    thrust->Kill();
    return EXIT_SUCCESS;
}
