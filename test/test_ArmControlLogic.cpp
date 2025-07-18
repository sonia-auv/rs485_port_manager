#include <gtest/gtest.h>
#include "rs485_port_manager/ArmControlLogic.hpp"

TEST(rs485_port_manager, motorsProcessingTest)
{
  std::cout << "Début du test" << std::endl;
  rs485_port_manager::ArmControlLogic ArmControlLogic1=rs485_port_manager::ArmControlLogic();

  // Test motor1 and motor2 in boundaries

  uint16_t motor1=500;
  uint16_t motor2=1500;

  std::cout << "motor1 : " << motor1 << std::endl;
  std::cout << "motor2 : " << motor2 << std::endl; 

  std::vector<uint8_t> data1;
  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  int16_t real_data1;
  int16_t real_data2;
  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  std::cout << "real_data1=" << real_data1 << std::endl;
  std::cout << "real_data2=" << real_data2 << std::endl;

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 500);
  ASSERT_EQ(real_data2, 1500);
  

  // Test max boundary for motor2
  motor1=600;
  motor2=2600;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 600);
  ASSERT_EQ(real_data2, 2500);

  // Test min boundary for motor2
  motor1=600;
  motor2=400;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 600);
  ASSERT_EQ(real_data2, 500);

  // Test min boundary for motor1
  motor1=2502;
  motor2=700;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 2500);
  ASSERT_EQ(real_data2, 700);

  // Test min boundary for motor1
  motor1=488;
  motor2=900;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 500);
  ASSERT_EQ(real_data2, 900);

  // Test min boundary for motor2 and max boundary for motor1
  motor1=2700;
  motor2=450;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 2500);
  ASSERT_EQ(real_data2, 500);

}


TEST(arm_port_manager, grabberProcessingTest)
{


  std::cout << "Début du test" << std::endl;
  rs485_port_manager::ArmControlLogic ArmControlLogic1=rs485_port_manager::ArmControlLogic();

  // Test grabber in boundaries
  float value= 50.6;

  std::cout << "value : " << value << std::endl;

  std::vector<uint8_t> data1;
  std::vector<float> grabber_values;
  data1=ArmControlLogic1.grabberProcessing(value);

  uint8_t cmd=data1[0];
  data1.erase(data1.begin());
  std::cout << data1.size()<< std::endl;
  rs485_port_manager::RS485Utils::convertBytesToFloat(data1,grabber_values,1);


  std::cout << "grabber_values[0]=" << grabber_values[0] << std::endl;

  ASSERT_EQ(cmd, ArmControlLogic1.CMD_GRABBER);
  ASSERT_EQ(grabber_values.at(0),value);

  // Test grabber max boundary 

  value= 207.7;

  data1=ArmControlLogic1.grabberProcessing(value);

  cmd=data1[0];
  data1.erase(data1.begin());
  std::cout << data1.size()<< std::endl;
  rs485_port_manager::RS485Utils::convertBytesToFloat(data1,grabber_values,1);

  ASSERT_EQ(cmd, ArmControlLogic1.CMD_GRABBER);
  ASSERT_EQ(grabber_values.at(1),100);

    // Test grabber max boundary 

    value= 7.7;

    data1=ArmControlLogic1.grabberProcessing(value);
  
    cmd=data1[0];
    data1.erase(data1.begin());
    std::cout << data1.size()<< std::endl;
    rs485_port_manager::RS485Utils::convertBytesToFloat(data1,grabber_values,1);
  
    ASSERT_EQ(cmd, ArmControlLogic1.CMD_GRABBER);
    ASSERT_EQ(grabber_values.at(2),10);

}

TEST(arm_port_manager, staticposTest)
{

  std::cout << "Début du test" << std::endl;
  rs485_port_manager::ArmControlLogic ArmControlLogic1=rs485_port_manager::ArmControlLogic();


  uint16_t motor1=500;
  uint16_t motor2=1500;
  uint8_t pos_wanted=0;

  std::cout << "motor1 : " << motor1 << std::endl;
  std::cout << "motor2 : " << motor2 << std::endl; 

  std::vector<uint8_t> data1;
  data1=ArmControlLogic1.staticPosProcessing(pos_wanted,motor1,motor2);

  int16_t real_data1;
  int16_t real_data2;
  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  std::cout << "real_data1=" << real_data1 << std::endl;
  std::cout << "real_data2=" << real_data2 << std::endl;

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 700);
  ASSERT_EQ(real_data2, 700);
  

  // Test static pos = repos
  pos_wanted=1;

  data1=ArmControlLogic1.staticPosProcessing(pos_wanted,motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, 700);
  ASSERT_EQ(real_data2, 500);

  // Test wrong static pos
  pos_wanted=10;


  data1=ArmControlLogic1.staticPosProcessing(pos_wanted,motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], ArmControlLogic1.CMD_MOTOR);
  ASSERT_EQ(real_data1, motor1);
  ASSERT_EQ(real_data2, motor2);
}

int main(int argc, char ** argv)
{
  try {
    std::cout << "Début du main" << std::endl;
    std::cout << "Suite de main " << std::endl;
    ::testing::InitGoogleTest(&argc, argv);
    std::cout << "Suite de main 2" << std::endl;
    int ret = RUN_ALL_TESTS();
    std::cout << "Suite de main 3" << std::endl;
    std::cout << "Suite de main 4" << std::endl;
    return ret;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception occurred" << std::endl;
  }
  return 1;
}