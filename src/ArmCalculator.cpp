#include <sstream>
#include "boost/log/trivial.hpp"
#include "rs485_port_manager/ArmCalculator.hpp"
#include <iostream>

using Eigen::MatrixXd;


// Class to compute the Direct and Inverse geometric models for a robotic arm with two joints ('RR') 

namespace rs485_port_manager
{
    ArmCalculator::ArmCalculator(){}

    ArmCalculator::~ArmCalculator(){}

    // (Direct model used to know the grabber final position after user provided both motors values) 
    std::vector<float> ArmCalculator::directGeometricModel(uint16_t motor1,uint16_t motor2)
    {
        float ratio_theta1=motor1/complete_rev_motor1*2*M_PI;
        float ratio_theta2=motor2/complete_rev_motor2*2*M_PI;

        std::cout << "((1250)/2500)*2*M_PI:\n" << (1/2) << std::endl;        
        std::cout << "ratio_theta1:\n" << ratio_theta1 << std::endl;        
        std::cout << "MAX_boundary_radius_mt1:\n" << MAX_boundary_radius_mt1 << std::endl;        
        std::cout << "ratio_theta2:\n" << ratio_theta2 << std::endl;        
        std::cout << "min_boundary_radius_mt1<=ratio_theta1:\n" << (min_boundary_radius_mt1<=ratio_theta1) << std::endl;        
        std::cout << "(ratio_theta1<=MAX_boundary_radius_mt1):\n" << (ratio_theta1<=MAX_boundary_radius_mt1) << std::endl;        
        std::cout << "(MAX_boundary_radius_mt1):\n" << (MAX_boundary_radius_mt1) << std::endl;        


        std::vector<float> data_effector_position;
        if ((min_boundary_radius_mt1<=ratio_theta1 && ratio_theta1<=MAX_boundary_radius_mt1)){

            //icirégler comment faire un calcul de matrice sur c++ et comment avoir cosinus et sinus
            std::vector<std::vector<float>> p3;
            std::vector<float> p3_line;
            static Eigen::Vector4f p3_1 = (Eigen::Vector4f() << 0,0,0,1).finished();
            std::cout << "Here is the initial matrix p3_1 :\n" << p3_1 << std::endl;        

            // First matrix transformation between 
            Eigen::Matrix4f T1;
            T1<<cos(ratio_theta1), -sin(ratio_theta1),0, l1*cos(ratio_theta1),
                sin(ratio_theta1), cos(ratio_theta1),0, l1*sin(ratio_theta1),
                0, 0, 1, 0,
                0, 0, 0, 1;
            std::cout << "Here is the initial matrix T1:\n" << T1 << std::endl;        
            Eigen::Matrix4f T2;
            T2 <<cos(ratio_theta2), -sin(ratio_theta2),0, l2*cos(ratio_theta2),
                sin(ratio_theta2), cos(ratio_theta2),0, l2*sin(ratio_theta2),
                0, 0, 1, 0,
                0, 0, 0, 1;
            Eigen::Vector4f first_iter=T2*p3_1;      
            Eigen::Vector4f effector_position=T1*T2*p3_1;

            std::cout << "Here is the initial matrix effector_position:\n" << effector_position << std::endl;        
            float x=effector_position(0,0);
            float y=effector_position(1,0);
            std::cout << "x:\n" << x << std::endl;        
            std::cout << "y:\n" << y << std::endl;   
        
            if (y>=0){
                data_effector_position.push_back(x);
                data_effector_position.push_back(y);
            }
            else {
                std::clog << "ERROR in motor2 or motor1 and 2 combined. HITTING THE SUB" << std::endl;
            }
        }
        else {
            std::clog << "ERROR in motor1. HITTING THE SUB" << std::endl;
        }

        return data_effector_position;
    }

    // (Inverse model used to know both motors values after user provided grabber final position desired) 
    std::vector<float> ArmCalculator::inverseGeometricModel(float x,float y,bool elbow_up)
    {

        // Test if the x and y are defining a position in the available workspace 
        /* Worksapce delimited by two nearly half (beacause of condition on y avoiding to hit the sub) circles : 
           inner nearly half-circle of (l1-l2) radius and outer nearly half-circle of (l1+l2) radius*/
        std::cout << "(x*x+y*y):\n" << (x*x+y*y) << std::endl;        
        std::cout << "(l1+l2)*(l1+l2):\n" << (l1+l2)*(l1+l2) << std::endl;        
        std::cout << "(l1-l2)*(l1-l2):\n" << (l1-l2)*(l1-l2) << std::endl;        
        std::vector<float> thetas;
        
        if ((x*x+y*y)<=(l1+l2)*(l1+l2) && (x*x+y*y)>=(l1-l2)*(l1-l2) && y>=0 ){  ////////////////////////////////////////////// condition sur y à modifier si nécessaire
            thetas=this->Testmotor1boundary(x,y,elbow_up);
            std::cout << "thetas[0]:\n" << thetas[0] << std::endl;        
            std::cout << "thetas[1]:\n" << thetas[1] << std::endl;   
            return thetas;
        }
        else {
            std::clog << "ERROR in x and y. Not in the workspace available. HITTING THE SUB" << std::endl;
            return thetas;
        }
    }    



    std::vector<float>  ArmCalculator::Testmotor1boundary(float x,float y,bool elbow_up){
        std::vector<float> thetas;
        float cos_theta2=(x*x+y*y-l1*l1-l2*l2)/(2*l1*l2);
    
        float sin_theta2;
        float theta2;
        if (elbow_up){
        // Elbow Up configuration
            sin_theta2=sqrt(1-cos_theta2*cos_theta2);
        }
        else{
        // Elbow Down configuration
            sin_theta2=- sqrt(1-cos_theta2*cos_theta2); 
            }
        
        theta2=atan2(sin_theta2,cos_theta2);



        float k1=l1+l2*cos(theta2);
        float k2=l2*sin(theta2);
        
        float theta1=atan2(y,x)-atan2(k2,k1);
        if (min_boundary_radius_mt1>theta1 or theta1>MAX_boundary_radius_mt1){
            thetas=this->Testmotor1boundary(x,y,!elbow_up);
        }
        else{
            thetas.push_back(theta1);
            thetas.push_back(theta2);
        }
        std::cout << "thetas[0]:\n" << thetas[0] << std::endl;        
        std::cout << "thetas[1]:\n" << thetas[1] << std::endl;        
        
        return thetas;

    }

    // Function used to compute a constant velocity trajectory between the initial and final position, accordding to joints values
    /* PARAMETERS : float *initial_setpoint = address of the first element of an array of 2 values containing 
                                                the initial joint setpoints of the robot in degrees    
                    float *final_setpoint = address of the first element of an array of 2 values containing the 
                                            final joint setpoints  of the robot in degrees                                       
                    float *v_max_arti = address of the first element of an array of 2 values containing the 
                                        maximum joint speeds in degrees per second                                   
                    float periode_in_s = value of the servo control period in seconds                                                 
                    float *trajectoire_articulaire = address of the first element of an array of 600.000 values 
                                                        where the calculated trajectory is stored. It will be 
                                                        stored in the following order:              
                            1) first setpoint for the first joint in degrees      
                            2) first setpoint for the second joint in degrees      
                            3) second setpoint for the first joint in degrees
                            4) second setpoint for the second joint in degrees, etc. */                                 
    // int ArmCalculator::interpole_joint_trajectory_constant_velocity(float *initial_setpoint, float* final_cmd,
    //         float* v_max_arti, float periode_in_s, 
    //         float *joint_traj, int *points_nb) {

    //     int nb_pts_total;
    //     int i=1;

    //     float tmin1, tmin2, tparcours, v1, v2;

    //     tmin1=abs(final_cmd[0]-initial_setpoint[0])/v_max_arti[0];
    //     tmin2=abs(final_cmd[1]-initial_setpoint[1])/v_max_arti[1];

    //     if(tmin1>tmin2)	tparcours=tmin1;
    //     else tparcours=tmin2;

    //     // Computing of the total number of points
    //     nb_pts_total = ceil(tparcours/periode_in_s);

    //     // Adjustment of travel time to account for rounding
    //     tparcours = nb_pts_total*periode_in_s ;

    //     if (nb_pts_total > 300000) 
    //         return -1; // ERROR CODE IF TOO MUCH POINTS

    //     else {
    //         v1=(final_cmd[0]-initial_setpoint[0])/tparcours;
    //         v2=(final_cmd[1]-initial_setpoint[1])/tparcours;

    //             *points_nb = nb_pts_total; // rangement du nb de points total dans l'adresse prévue

    //         for(i=0;i<nb_pts_total;i++)
    //         {
    //             joint_traj[2*i]=initial_setpoint[0]+(i+1)*periode_in_s*v1;
    //             joint_traj[2*i+1]=initial_setpoint[1]+(i+1)*periode_in_s*v2;
    //         }	

    //         return 0;

    //     }

    // }
    //////////////////////////////////////////////// ARM SYSTEM IS CALLING THIS FUNCTION AND directGeometricModel BEFORE
    ////////////////////////////////////////////////PROBLEME : TRANSMETTRE PLUSIEURS MOTORS VALUES POUR PUBLISHER//////////////////////////////////////////////

    // int ArmCalculator::interpole_trajectoire_cartesienne_v_constante(float *joint_initial_setpoint, float* cartesian_final_setpoint,
    //                             float* v_max_cartes, float periode_in_s, 
    //                         float *joint_traj, int *points_nb) {

    //     int nb_pts_total;
    //     float l1 = 285.0;
    //     float l2 = 185.0;
    //     float tmin1;
    //     float tmin2;
    //     float tparcours;
    //     float trajectoire_carte[2];
    //     float consigne_init_cartes[2];
    //     float mgi[2];
        
    //     this->inverseGeometricModel(joint_initial_setpoint,consigne_init_cartes);
    //     float v1,v2;
        

    //     tmin1=abs(cartesian_final_setpoint[0]-joint_initial_setpoint[0])/v_max_cartes[0];
    //     tmin2=abs(cartesian_final_setpoint[1]-joint_initial_setpoint[1])/v_max_cartes[1];
    
    //     if(tmin1>tmin2)	tparcours=tmin1;
    //     else tparcours=tmin2;
    
    //     // calcul du nombre de points au total
    //     nb_pts_total = ceil(tparcours/periode_in_s); // à remplacer par le vrai calcul
    //     // ajustement du temps de parcours pour tenir compte de l'arrondi
    //     tparcours = nb_pts_total*periode_in_s ;

    //     if (nb_pts_total > 300000) 
    //             return -1; // CODE D'ERREUR POUR TROP DE POINTS

    //     else {
    //         *points_nb = nb_pts_total; // rangement du nb de points total dans l'adresse prévue
    //         if( this->inverseGeometricModel(cartesian_final_setpoint,trajectoire_articulaire).size() == 0){
    //             // le mgi a renvoyé une erreur : on ne bouge pas
    //             trajectoire_articulaire[0]=joint_initial_setpoint[0];
    //             trajectoire_articulaire[1]=joint_initial_setpoint[1];
    //             return -2; //CODE POUR ERREUR DE MGI EN COURS DE TRAJ
    //         }
    //         else {
    //             v1=(cartesian_final_setpoint[0]-joint_initial_setpoint[0])/tparcours;
    //             v2=(cartesian_final_setpoint[1]-joint_initial_setpoint[1])/tparcours;
    //             int i=0;
    //             for ( i=0; i<nb_pts_total;i++){
    //                 //printf("\nA");
    //                 float mgi[2];
    //                 trajectoire_carte[0]=joint_initial_setpoint[0]+(i+1)*periode_in_s*v1;
    //                 trajectoire_carte[1]=joint_initial_setpoint[1]+(i+1)*periode_in_s*v2;
    //                 modele_geometrique_inverse(trajectoire_carte,mgi);
    //                 trajectoire_articulaire[2*i]=mgi[0];
    //                 trajectoire_articulaire[2*i+1]=mgi[1];
    //                 //printf("\nB");
                    
    //             }
    //         }			
    //     }
    //     return 0; // CODE DE RETOUR POUR TOUT VA BIEN
    // }

} // end namespace



