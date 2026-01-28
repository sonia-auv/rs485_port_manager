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
#include "sonia_common_ros2/msg/rs485msg.hpp"

#include "CommonDefinitionRS485.hpp"
#include "InterfaceModuleRS485.hpp"
#include <vector>

namespace rs485_port_manager
{

    class RS485Driver : public rclcpp::Node
    {
        public:
        /**
         *  function to get the instance of RS485Driver
         */
        static RS485Driver *GetInstance();
        ~RS485Driver() = default;

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

        /**
         * @brief add Message to send
         */
        void AddMessage(queueObject msg);

        /**
         * @brief Start System
         */
        void Start();

        /**
         * @brief Add Observateur
         */
        void AddObservateur(rs485_port_manager::InterfaceModuleRS485 *interfaceModule);

        private:
        static RS485Driver *_instance;
        RS485Driver();

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

        void messageRS485CallBack(const sonia_common_ros2::msg::RS485msg &msg);

        static const int _DATA_READ_CHUNCK = 1024;
        const u_int8_t _START_BYTE = 0x3A;
        const u_int8_t _END_BYTE = 0x0D;
        const uint8_t _GET_KILL_STATUS_MSG[8] = {0x3A, 4, 1, 1, 0, 0, 77, 0x0D};
        const uint8_t _GET_MISSION_STATUS_MSG[8] = {0x3A, 4, 0, 1, 1, 0, 77, 0x0D};
        const uint8_t _GET_POWER_MSG[15] = {0x3A, 8, 0, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 95, 0x0D};
        const uint8_t _GET_FEEDBACK_MSG[15] = {0x3A, 8, 15, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 110, 0x0D};
        const uint8_t _EXPECTED_PWR_VOLT_SIZE = 10;

        std::vector<rs485_port_manager::InterfaceModuleRS485*> ObservateurInterfaceModule;

        sonia_common_cpp::SerialConn _rs485Connection;

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
    };

}