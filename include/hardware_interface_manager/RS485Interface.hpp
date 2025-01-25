#pragma once

#include <stdio.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <tuple>
#include <vector>

#include "SharedQueue.hpp"
#include "sonia_common_cpp/SerialConn.hpp"
#include "sonia_common_ros2/msg/battery_power_messages.hpp"
#include "sonia_common_ros2/msg/kill_status.hpp"
#include "sonia_common_ros2/msg/mission_status.hpp"
#include "sonia_common_ros2/msg/motor_feedback.hpp"
#include "sonia_common_ros2/msg/motor_power_messages.hpp"
#include "sonia_common_ros2/msg/motor_pwm.hpp"
#include "sonia_common_ros2/msg/serial_message.hpp"
#include "sonia_common_ros2/srv/dropper_service.hpp"


namespace sonia_hw_interface
{
    /**
     * @brief Internal Queue Object
     *
     */
    struct queueObject
    {
        uint8_t slave;
        uint8_t cmd;
        std::vector<uint8_t> data;
    };

    class RS485Interface : public rclcpp::Node
    {
        public:
        RS485Interface();
        ~RS485Interface();

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
         * @brief Slave Ids
         *
         */
        enum _SlaveId : uint8_t
        {
            SLAVE_PSU0 = 0,  // AUV7 Only
            SLAVE_PSU1 = 1,  // AUV7 Only
            SLAVE_PSU2 = 2,  // AUV7 Only
            SLAVE_PSU3 = 3,  // AUV7 Only
            SLAVE_KILLMISSION = 4,
            SLAVE_ESC = 5,
            SLAVE_IO = 6,
            SLAVE_STATE_SCREEN = 7,
            SLAVE_PWR_MANAGEMENT = 8,  // AUV8 Only
        };

        /**
         * @brief Command Ids
         *
         */
        enum _Cmd : uint8_t
        {
            CMD_MISSION = 0,
            CMD_KILL = 1,
            CMD_VOLTAGE = 0,
            CMD_CURRENT = 1,
            CMD_TEMPERATURE = 2,
            CMD_READ_MOTOR = 15,
            CMD_ACT_MOTOR = 16,
            CMD_PWM = 17,
            CMD_IO_TEMP = 0,
            CMD_IO_DROPPER_ACTION = 1,
            CMD_IO_TORPEDO_ACTION = 2,
            CMD_IO_ARM_ACTION = 3,
            CMD_IO_LEAK_SENSOR = 4,
            CMD_KEEP_ALIVE = 30,
        };

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
        void pollPower();

        /**
         * @brief Processes a dropper service request.
         *
         * @param request
         * @param response
         */
        void processDropperRequest(const std::shared_ptr<sonia_common_ros2::srv::DropperService::Request> request,
                                   std::shared_ptr<sonia_common_ros2::srv::DropperService::Response> response);

        void processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data);

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

        void publishMotor(uint8_t cmd, std::vector<float> data);

        void publishBattery(uint8_t cmd, float *data);

        void publishMotorFeedback(std::vector<uint8_t> data);

        void EnableDisableMotors(const std_msgs::msg::Bool &msg);
        void ToggleMotors(const bool state, uint8_t size, std::vector<uint8_t> &data);
        void PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg);

        int convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size);

        union _bytesToFloat
        {
            uint8_t bytes[4];
            float_t value;
        };

        const uint8_t nb_thruster = 8;
        const uint8_t nb_battery = 2;
        static const int _DATA_READ_CHUNCK = 1024;
        const u_int8_t _START_BYTE = 0x3A;
        const u_int8_t _END_BYTE = 0x0D;
        const uint8_t _GET_KILL_STATUS_MSG[8] = {0x3A, 4, 1, 1, 0, 0, 77, 0x0D};
        const uint8_t _GET_MISSION_STATUS_MSG[8] = {0x3A, 4, 0, 1, 1, 0, 77, 0x0D};
        const uint8_t _GET_POWER_MSG[15] = {0x3A, 8, 0, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 95, 0x0D};
        const uint8_t _GET_FEEDBACK_MSG[15] = {0x3A, 8, 15, 8, 1, 1, 1, 1, 1, 1, 1, 1, 0, 110, 0x0D};
        const uint8_t _EXPECTED_PWR_VOLT_SIZE = 10;

        sonia_common_cpp::SerialConn _rs485Connection;


        rclcpp::Publisher<sonia_common_ros2::msg::KillStatus>::SharedPtr _publisherKill;
        rclcpp::Publisher<sonia_common_ros2::msg::MissionStatus>::SharedPtr _publisherMission;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorVoltages;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryVoltages;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorCurrents;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryCurrents;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPowerMessages>::SharedPtr _publisherMotorTemperature;
        rclcpp::Publisher<sonia_common_ros2::msg::BatteryPowerMessages>::SharedPtr _publisherBatteryTemperature;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorFeedback>::SharedPtr _publisherMotorFeedback;
        rclcpp::Publisher<sonia_common_ros2::msg::MotorPwm>::SharedPtr _publisherThrusterPwm;
        rclcpp::Subscription<sonia_common_ros2::msg::MotorPwm>::SharedPtr _subscriberThrusterPwm;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriberMotorOnOff;
        rclcpp::Service<sonia_common_ros2::srv::DropperService>::SharedPtr _dropperServer;
        rclcpp::TimerBase::SharedPtr _timerKillMission;
        rclcpp::TimerBase::SharedPtr _timerPowerRequest;

        std::thread _reader;
        std::thread _parser;
        std::thread _writer;

        rclcpp::CallbackGroup::SharedPtr group1;
        SharedQueue<queueObject> _writerQueue;
        SharedQueue<uint8_t> _parseQueue;
        std::mutex mutex_;


        bool _thread_control;

        uint8_t ESC_SLAVE;

        const char *auv;
    };

}  // namespace sonia_hw_interface