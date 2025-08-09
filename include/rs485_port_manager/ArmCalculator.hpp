#pragma once
#include<Eigen/Dense>
#include <cmath>



#define ID1 "ISDPT"
using namespace std;

namespace rs485_port_manager
{

    class ArmCalculator
    {
        public:
            ArmCalculator();
            ~ArmCalculator();

            std::vector<float> directGeometricModel(uint16_t motor1,uint16_t motor2);
            std::vector<float> inverseGeometricModel(float x,float y, bool elbow_down);
            std::vector<float> Testmotor1boundary(float x,float y,bool elbow_up);

            int interpole_joint_trajectory_constant_velocity(float *initial_setpoint, float* final_cmd,float* v_max_arti, float periode_in_s, float *joint_traj, int *points_nb);
            int interpole_trajectoire_cartesienne_v_constante(float *joint_initial_setpoint, float* cartesian_final_setpoint,float* v_max_cartes, float periode_in_s, float *joint_traj, int *points_nb);

            float l1=0.324;//////////////////////////////////////////////à modifier 
            float l2=0.1;//////////////////////////////////////////////à modifier 
            float complete_rev_motor1=2500;//////////////////////////////////////////////à modifier si nécessaire
            float complete_rev_motor2=2500;//////////////////////////////////////////////à modifier si nécessaire
        
        private:

            // Boundaries for motors values to avoid hitting the sub
            float min_boundary_radius_mt1=0;//////////////////////////////////////////////à modifier si nécessaire
            float MAX_boundary_radius_mt1=M_PI;//////////////////////////////////////////////à modifier si nécessaire


    };
}