#pragma once

#include <stdio.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <tuple>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "sonia_common_cpp/SharedQueue.hpp"
#include "sonia_common_cpp/SerialConn.hpp"
#include "sonia_common_ros2/msg/kill_status.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"
#include "sonia_common_ros2/msg/serial_message.hpp"
#include "sonia_common_ros2/msg/rs485msg.hpp"
#include "sonia_common_ros2/srv/actuator_service.hpp"

#include "Definition.hpp"


namespace rs485_port_manager
{

    class RS485Provider : public rclcpp::Node
    {
        public:
        RS485Provider();
        ~RS485Provider();

        /**
         * @brief Open designated internal serial port.
         *
         * @return bool True if opened successfully else False.
         */
        bool OpenPort();

        /**
         * @brief Kill all internal threads.
         */
        void Kill();

        private:

        /**
         * @brief Calculate Checksum.
         *
         * @param slave     Slave ID
         * @param cmd       Command ID
         * @param nbByte    Length of Data
         * @param data      Data Vector
         * @return std::tuple<uint8_t, uint8_t> Checksum over 2 bytes.
         */
        std::tuple<uint8_t, uint8_t> checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data);

        /**
         * @brief read data from RS485 connection and pushes into the parse queue.
         * This function runs in a loop, continuously reading data from the RS485 connection in chunks.
         * The data is then pushed into a parse queue for further processing.
         */
        void readData();

        /**
         * @brief write data to RS485 bassed on message from _writerQueue.
         *
         */
        void writeData();

        /**
         * @brief parse Data Continuously received from RS485 connection.
         * Loops continuously parsing data from _parseQueue and acts based on the messages received.
         */
        void parseData();

        /**
         * @brief Polls the status of kill and mission via RS485 connection.
         *
         * This function sends requests to retrieve the status of kill and mission
         * via the RS485 connection.
         */
        void pollKillMission();

        /**
         * @brief Processes a actuator service request.
         *
         * @param request
         * @param response
         */
        void processActuatorRequest(const std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Request> request,
                                   std::shared_ptr<sonia_common_ros2::srv::ActuatorService::Response> response);

        /**
         * @brief
         *
         * @param status
         */
        void publishKill(bool status);

        /**
         * @brief
         *
         * @param status
         */
        void publishMission(bool status);
        static const int _DATA_READ_CHUNCK = 1024;
        const u_int8_t _START_BYTE = 0x3A;
        const u_int8_t _END_BYTE = 0x0D;
        const uint8_t _GET_KILL_STATUS_MSG[8] = {0x3A, 4, 1, 1, 0, 0, 77, 0x0D};
        const uint8_t _GET_MISSION_STATUS_MSG[8] = {0x3A, 4, 0, 1, 1, 0, 77, 0x0D};
        const uint8_t _GET_POWER_MSG[15] = {0x3A, 8, 0, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 95, 0x0D};
        const uint8_t _GET_FEEDBACK_MSG[15] = {0x3A, 8, 15, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 110, 0x0D};
        const uint8_t _EXPECTED_PWR_VOLT_SIZE = 10;

        sonia_common_cpp::SerialConn _rs485Connection;

        rclcpp::Service<sonia_common_ros2::srv::ActuatorService>::SharedPtr _actuatorService;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
        rclcpp::TimerBase::SharedPtr _timerPowerRequest;

        std::thread _reader;
        std::thread _parser;
        std::thread _writer;

        std::mutex _mtxParser;
        std::condition_variable _cvReaderParser;

        std::mutex _mtxWriter;
        std::condition_variable _cvReaderWriter;
        sonia_common_cpp::SharedQueue<queueObject> _writerQueue;
        sonia_common_cpp::SharedQueue<uint8_t> _parseQueue;

        bool _thread_control;

        // all needed for the rework and the split

        rclcpp::Subscription<sonia_common_ros2::msg::RS485msg>::SharedPtr _subscriptionRS485;

        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publisherKill;
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publisherIO;
        rclcpp::Publisher<sonia_common_ros2::msg::RS485msg>::SharedPtr _publisherMotor;

        void RS485callback(const sonia_common_ros2::msg::RS485msg &msg);
    };

}