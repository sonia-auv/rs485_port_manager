#pragma once

#include <stdint.h>

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