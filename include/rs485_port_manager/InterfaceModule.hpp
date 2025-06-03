#pragma once

#include <stdio.h>
#include <rclcpp/rclcpp.hpp>

namespace interface_module
{

    class InterfaceModule : public rclcpp::Node
    {
        public:

        /**
         * @brief Kill all internal threads.
         */
        void Kill();

        private:
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

            int convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size);
    }

}