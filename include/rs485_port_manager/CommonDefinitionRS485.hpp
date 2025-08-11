#pragma once

#include <stdint.h>

namespace rs485_port_manager
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

        void printTram()
        {
            printf("%x ", slave);
            printf("%x ", cmd);
            for (size_t i = 0; i < data.size(); i++)
            {
                printf("%x ", data[i]);
            }
            
            printf("\n");
        }
    };

    /**
     * @brief Slave Ids
     *
     */
    enum SlaveId : uint8_t
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
    enum Cmd : uint8_t
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
        CMD_IO_ARM_GRABBER=3,
        CMD_IO_LEAK_SENSOR = 4,
        CMD_IO_ARM_MOTOR=5,
        CMD_KEEP_ALIVE = 30,
    };

    union bytesToFloat
    {
        uint8_t bytes[4];
        float value;
    };

    class RS485Utils {
        public:
        static int convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size) // because it's a static function can be called without creating a RS485Utils object
        {
            uint8_t size_req = req.size();
            if (size_req % 4 != 0) return -1;
    
            bytesToFloat converter;
    
            for (uint8_t i = 0; i < size; ++i)  // shifting of 4 for each data
            {
                converter.bytes[0] = req[4 * i];
                converter.bytes[1] = req[4 * i + 1];
                converter.bytes[2] = req[4 * i + 2];
                converter.bytes[3] = req[4 * i + 3];
                res.push_back(converter.value);
            }
            return 0;
        };
    };



    
}