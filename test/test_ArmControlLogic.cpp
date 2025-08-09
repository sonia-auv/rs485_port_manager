#include <gtest/gtest.h>
#include "rs485_port_manager/ArmControlLogic.hpp"
#include "rs485_port_manager/ArmCalculator.hpp"

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

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 500);
  ASSERT_EQ(real_data2, 1500);
  

  // Test max boundary for motor2
  motor1=600;
  motor2=2600;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 600);
  ASSERT_EQ(real_data2, 2500);

  // Test min boundary for motor2
  motor1=600;
  motor2=400;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 600);
  ASSERT_EQ(real_data2, 500);

  // Test min boundary for motor1
  motor1=2502;
  motor2=700;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 2500);
  ASSERT_EQ(real_data2, 700);

  // Test min boundary for motor1
  motor1=488;
  motor2=900;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 500);
  ASSERT_EQ(real_data2, 900);

  // Test min boundary for motor2 and max boundary for motor1
  motor1=2700;
  motor2=450;

  data1=ArmControlLogic1.motorsProcessing(motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 2500);
  ASSERT_EQ(real_data2, 500);

}


TEST(rs485_port_manager, grabberProcessingTest)
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

  ASSERT_EQ(cmd, rs485_port_manager::Cmd::CMD_GRABBER);
  ASSERT_EQ(grabber_values.at(0),value);

  // Test grabber max boundary 

  value= 207.7;

  data1=ArmControlLogic1.grabberProcessing(value);

  cmd=data1[0];
  data1.erase(data1.begin());
  std::cout << data1.size()<< std::endl;
  rs485_port_manager::RS485Utils::convertBytesToFloat(data1,grabber_values,1);

  ASSERT_EQ(cmd, rs485_port_manager::Cmd::CMD_GRABBER);
  ASSERT_EQ(grabber_values.at(1),100);

    // Test grabber max boundary 

    value= 7.7;

    data1=ArmControlLogic1.grabberProcessing(value);
  
    cmd=data1[0];
    data1.erase(data1.begin());
    std::cout << data1.size()<< std::endl;
    rs485_port_manager::RS485Utils::convertBytesToFloat(data1,grabber_values,1);
  
    ASSERT_EQ(cmd, rs485_port_manager::Cmd::CMD_GRABBER);
    ASSERT_EQ(grabber_values.at(2),value);

}

TEST(rs485_port_manager, staticposTest)
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

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 700);
  ASSERT_EQ(real_data2, 700);
  

  // Test static pos = repos
  pos_wanted=1;

  data1=ArmControlLogic1.staticPosProcessing(pos_wanted,motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, 700);
  ASSERT_EQ(real_data2, 500);

  // Test wrong static pos
  pos_wanted=10;


  data1=ArmControlLogic1.staticPosProcessing(pos_wanted,motor1,motor2);

  real_data1=static_cast<unsigned>(data1[1]) << 8 | static_cast<unsigned>(data1[2]);
  real_data2=static_cast<unsigned>(data1[3]) << 8 | static_cast<unsigned>(data1[4]);

  ASSERT_EQ(data1[0], rs485_port_manager::Cmd::CMD_MOTOR);
  ASSERT_EQ(real_data1, motor1);
  ASSERT_EQ(real_data2, motor2);
}

TEST(rs485_port_manager,directGeometricModelTest){
  std::cout << "Début du test" << std::endl;
  rs485_port_manager::ArmCalculator armcalculator=rs485_port_manager::ArmCalculator();

  // Test motor1 and 2 in boundaries
  uint16_t motor1= 500;
  uint16_t motor2= 500;


  std::vector<float> data_direct;
  data_direct=armcalculator.directGeometricModel(motor1,motor2);

  std::cout << "data size=" <<data_direct.size()<< std::endl;

  std::cout << "data_direct[0]=" << data_direct[0] << std::endl;
  std::cout << "data_direct[1]=" << data_direct[1] << std::endl;

  float ratio_m1=motor1/armcalculator.complete_rev_motor1*2*M_PI;
  float ratio_m2 =motor2/armcalculator.complete_rev_motor2*2*M_PI;

  std::vector<float> data_inv;
  bool elbow_up=true;
  data_inv=armcalculator.inverseGeometricModel(data_direct[0],data_direct[1],elbow_up);

  std::cout << "data_inv[0]=" << data_inv[0] << std::endl;
  std::cout << "data_inv[1]=" << data_inv[1] << std::endl;

  ASSERT_EQ(data_direct[0],armcalculator.l2*cos(ratio_m1+ratio_m2)+armcalculator.l1*cos(ratio_m1));
  ASSERT_EQ(data_direct[1],armcalculator.l2*sin(ratio_m1+ratio_m2)+armcalculator.l1*sin(ratio_m1));



  // Test motor2 outside of boundaries
  std::cout << "Test motor2 outside of boundaries\n" <<data_direct.size()<< std::endl;

  motor1= 0;
  motor2= 2000;


  data_direct=armcalculator.directGeometricModel(motor1,motor2);

  std::cout << "data size=" <<data_direct.size()<< std::endl;
  std::cout << "data size=" <<data_direct.size()<< std::endl;

  // data_inv=armcalculator.inverseGeometricModel(data_direct[0],data_direct[1],elbow_up);

  // std::cout << "data_inv[0]=" << data_inv[0] << std::endl;
  // std::cout << "data_inv[1]=" << data_inv[1] << std::endl;  

  ASSERT_EQ(data_direct.size(),0);

  std::cout << "Test motor1 outside of boundaries\n" <<data_direct.size()<< std::endl;

  // Test motor1 outside of boundaries
  motor1= 1300;
  motor2= 1625;


  data_direct=armcalculator.directGeometricModel(motor1,motor2);

  std::cout << "data size=" <<data_direct.size()<< std::endl;

  // data_inv=armcalculator.inverseGeometricModel(data_direct[0],data_direct[1],!elbow_up);

  // std::cout << "data_inv[0]=" << data_inv[0] << std::endl;
  // std::cout << "data_inv[1]=" << data_inv[1] << std::endl;

  ASSERT_EQ(data_direct.size(),0);
}

TEST(rs485_port_manager,inverseGeometricModelTest){
  std::cout << "Début du test" << std::endl;
  rs485_port_manager::ArmCalculator armcalculator=rs485_port_manager::ArmCalculator();

  // Effector position in boundaries
  float x= 0.424;
  float y= 0;
  bool elbow_up=true;


  std::vector<float> data_inv;
  data_inv=armcalculator.inverseGeometricModel(x,y,elbow_up);

  std::cout << "data size=" <<data_inv.size()<< std::endl;

  std::cout << "data_inv[0]=" << data_inv[0] << std::endl;
  std::cout << "data_inv[1]=" << data_inv[1] << std::endl;
  ASSERT_EQ(data_inv[0],0);
  ASSERT_EQ(data_inv[1],0);

  uint16_t motor1=data_inv[0]*2500/(2*M_PI);
  uint16_t motor2=data_inv[1]*2500/(2*M_PI);
  std::vector<float> data_direct;

  data_direct=armcalculator.directGeometricModel(motor1,motor2);
  std::cout << "data_direct1[0]=" << data_direct[0] << std::endl;
  std::cout << "data_direct1[1]=" << data_direct[1] << std::endl;

  // Effector position in boundaries
  float line_effector_origin=sqrt(armcalculator.l1*armcalculator.l1+armcalculator.l2*armcalculator.l2);
  x=line_effector_origin*cos(acos(armcalculator.l1/line_effector_origin)+M_PI/4);
  y=line_effector_origin*sin(acos(armcalculator.l1/line_effector_origin)+M_PI/4);

  float X=acos(x/line_effector_origin)-acos(armcalculator.l1/line_effector_origin);
  std::cout << "x2=" << x << std::endl;
  std::cout << "y2=" << y << std::endl;
  std::cout << "X=" << X << std::endl;

  elbow_up=true;

  data_inv=armcalculator.inverseGeometricModel(x,y,elbow_up);

  std::cout << "data size=" <<data_inv.size()<< std::endl;

  std::cout << "data_inv[0]=" << data_inv[0] << std::endl;
  std::cout << "data_inv[1]=" << data_inv[1] << std::endl;

  motor1=data_inv[0]*2500/(2*M_PI);
  motor2=data_inv[1]*2500/(2*M_PI);
  data_direct=armcalculator.directGeometricModel(motor1,motor2);
  std::cout << "data_direct[0]=" << data_direct[0] << std::endl;
  std::cout << "data_direct[1]=" << data_direct[1] << std::endl;

  ASSERT_EQ(data_inv[0],M_PI/4);
  ASSERT_EQ(data_inv[1],M_PI/2);


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