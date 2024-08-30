#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <string>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"

// Includes for the TCP/IP socket connection
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>


// // this code is only used for automatica 
int main(int argc, char** argv) {

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try
    {
//         // Create robot instance
        franka::Robot myRobot(argv[1],franka::RealtimeConfig::kIgnore);

//         // Create the gripper instance
         franka::Gripper gripper(argv[1]);

//         // Setting the collision behavior appropriate for hands-on use
        myRobot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        

//         // Set speedfactor
         double speedfactor = 0.3;
//        

          std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
 //         gripper.move(0.1, 0.1);




         MotionGenerator motiongenerator(speedfactor, safepos);



            // gripper.homing();
             myRobot.control(motiongenerator);
            // gripper. move(0.03,0.02);
            gripper.move(0.04, 0.1);

         std::cout << "Reached initial position" << std::endl;


//         // 1: Move down to pick something up
          std::cout << "First motion: Move down to the bottle position pick the bottle up"  << std::endl;
          double time1 = 0.0;
          double finish_time1 = 3.0;

//         // at the initial position: echo_robot_state 192.168.3.100
//         // find the variable "O_T_EE"
//         // this is the initial_pos1 
          std::array<double, 16> initial_pos1; 
        

         auto cartesian_initial_motion_1 = [&initial_pos1, &time1, &finish_time1](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {
            std::cout <<rstate.O_T_EE_c[14] << std::endl;
            time1 += periode.toSec();

            if(time1==0.0)
            {
                initial_pos1 = rstate.O_T_EE_c;
                
            }


             constexpr double kRadius = 0.4;
             double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time1));
             double dx = 0.08 * (cos(M_PI_4*(1-cos(M_PI/3*time1)))-1);
             double dy = 0.26 * (cos(M_PI_4*(1-cos(M_PI/3*time1)))-1);
             double dz = kRadius * (cos(M_PI_4*(1-cos(M_PI/3*time1)))-1);
            std::array<double, 16> current_pos = initial_pos1;
            current_pos[12] += dx;
            current_pos[13] += dy;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if(time1 >= finish_time1)
            {
                std::cout << "End moving to the position" << std::endl;
                std::cout << rstate << std::endl;
                return  franka::MotionFinished(output);
            }
 
            return output;

         };

           myRobot.control(cartesian_initial_motion_1);
           std::cout << "Reached position 2" << std::endl; 


           
//         // 2: Grasp
//         // First open the gripper
           std::cout << "2. Open gripper"  << std::endl;
           
           gripper.grasp(0.01, 0.02, 70);
           std::cout << " Finished grasping"  << std::endl;

        
        
//         // 3. Move up
            std::cout << "3. Move up"  << std::endl;
         double time_2 = 0.0;
         double finish_time_2 = 3.0;
         std::array<double, 16> initial_pos_2;

         auto cartesian_moving_slightlyUp = [&initial_pos_2, &time_2, &finish_time_2](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {

            time_2 += periode.toSec();


            if(time_2==0.0)
            {
                initial_pos_2 = rstate.O_T_EE_c;
            }



               double dz = -0.105*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1);

               std::array<double, 16> current_pos = initial_pos_2;
            
               current_pos[14] += dz;

               franka::CartesianPose output = current_pos;

              if(time_2 >= finish_time_2)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                std::cout << "t:" << time_2 << "  periode: " << periode.toSec() << " z-Koord.: " << rstate.O_T_EE_c[14] << std::endl;




                return output;
          };

          myRobot.control(cartesian_moving_slightlyUp);
        //   myRobot.control(motiongenerator);


        //    std::array<double, 7> initial_position_4;
        //     double time_41 = 0.0;
        //     myRobot.control([&initial_position_4, &time_41](const franka::RobotState& robot_state,
        //                                             franka::Duration period) -> franka::JointPositions {
        //     time_41 += period.toSec();
        //     if (time_41== 0.0) {
        //         initial_position_4= robot_state.q_d;
        //     }
        //     double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 5 * time_41));
        //     franka::JointPositions output = {{initial_position_4[0]+2.3*delta_angle , initial_position_4[1],
        //                                         initial_position_4[2], initial_position_4[3],
        //                                         initial_position_4[4] , initial_position_4[5],
        //                                         initial_position_4[6]}};
        //     if (time_41 >= 5) {
        //         std::cout << std::endl << "Finished the 4th motion" << std::endl;
        //         std::cout  << robot_state  << std::endl;
        //         return franka::MotionFinished(output);
        //     }
        //     return output;

        //     });

 //  4. Move around
             std::cout << "4. Move around"  << std::endl;
           double time_3 = 0.0;
           double finish_time_3 = 3.0;
           std::array<double, 16> initial_pos_3; 

            auto cartesian_moving_around = [&initial_pos_3, &time_3, &finish_time_3](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
            {

                time_3 += periode.toSec();


                if(time_3==0.0)
                {
                    initial_pos_3 = rstate.O_T_EE_c;
                }


             
           
               
                //double dz = -0.26*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1);
                double dx = -0.08 * (cos(M_PI_4*(1-cos(M_PI/3*time_3)))-1);
                double dy = -0.26 * (cos(M_PI_4*(1-cos(M_PI/3*time_3)))-1);
                double dz = -0.295*(cos(M_PI_4*(1-cos(M_PI/3*time_3)))-1);
               std::array<double, 16> current_pos = initial_pos_3;
               current_pos[12] += dx;
               current_pos[13] += dy;
               current_pos[14] += dz;

               franka::CartesianPose output = current_pos;

                if(time_3 >= finish_time_3)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                


                return output;
                
            };

            myRobot.control(cartesian_moving_around);

//    //4. 2 + Move around
             std::cout << "4.2 Move around"  << std::endl;
           double time_32 = 0.0;
           double finish_time_32 = 4.0;
           std::array<double, 16> initial_pos_32; 

            auto cartesian_moving_around2 = [&initial_pos_32, &time_32, &finish_time_32](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
            {

                time_32 += periode.toSec();


                if(time_32==0.0)
                {
                    initial_pos_32 = rstate.O_T_EE_c;
                }



                 double dx = 0.445 * (cos(M_PI_4*(1-cos(M_PI/4*time_32)))-1);
                double dy = -0.325 * (cos(M_PI_4*(1-cos(M_PI/4*time_32)))-1);
                double dz = 0.1*(cos(M_PI_4*(1-cos(M_PI/4*time_32)))-1);
               std::array<double, 16> current_pos = initial_pos_32;
               current_pos[12] += dx;
               current_pos[13] += dy;
               current_pos[14] += dz;
               double dphi = 0.9*(cos(M_PI_4*(1-cos(M_PI/4*time_32)))-1);
                current_pos[0] += dphi;
                current_pos[1] -= dphi;
                current_pos[4] -= dphi;
                current_pos[5] -= dphi;

               franka::CartesianPose output = current_pos;

                if(time_32 >= finish_time_32)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                


                return output;
                
            };

            myRobot.control(cartesian_moving_around2);

//    //4. 3 + Move around
             std::cout << "4.3 Move around"  << std::endl;
           double time_33 = 0.0;
           double finish_time_33 = 4.0;
           std::array<double, 16> initial_pos_33; 

            auto cartesian_moving_around3 = [&initial_pos_33, &time_33, &finish_time_33](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
            {

                time_33 += periode.toSec();


                if(time_33==0.0)
                {
                    initial_pos_33 = rstate.O_T_EE_c;
                }



                 double dx = 0.445 * (cos(M_PI_4*(1-cos(M_PI/4*time_33)))-1);
                double dy = 0.1 * (cos(M_PI_4*(1-cos(M_PI/4*time_33)))-1);
                double dz = -0.005*(cos(M_PI_4*(1-cos(M_PI/4*time_33)))-1);
               std::array<double, 16> current_pos = initial_pos_33;
               current_pos[12] += dx;
               current_pos[13] += dy;
               current_pos[14] += dz;

               franka::CartesianPose output = current_pos;

                if(time_33 >= finish_time_33)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                


                return output;
                
            };

            myRobot.control(cartesian_moving_around3);
//         // 4: place object

           double time_4 = 0.0;
           double finish_time_4 = 3.0;
          std::array<double, 16> initial_pos_4;

        auto cartesian_moving_slightlyDown = [&initial_pos_4, &time_4, &finish_time_4 ](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose{

           time_4 += periode.toSec();


           if(time_4==0.0)
           {
               initial_pos_4 = rstate.O_T_EE_c;
           }


               double dz = 0.2*(cos(M_PI_4*(1-cos(M_PI/3*time_4)))-1);

              std::array<double, 16> current_pos = initial_pos_4;
//            //current_pos[12] += dx;
              current_pos[14] += dz;

             franka::CartesianPose output = current_pos;

                if(time_4 >= finish_time_4)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }



                return output;
                };

          myRobot.control(cartesian_moving_slightlyDown);

//        // 5. Open the gripper
          gripper.move(0.05, 0.03);
          //gripper.stop();
//         // 6. Move up
         std::cout << "6. Move up"  << std::endl;
         double time_6= 0.0;
         double finish_time_6 = 3.0;
         std::array<double, 16> initial_pos_6;

         auto cartesian_moving_Up = [&initial_pos_6, &time_6, &finish_time_6](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {

            time_6 += periode.toSec();


            if(time_6==0.0)
            {
                initial_pos_6 = rstate.O_T_EE_c;
            }



               double dz = -0.25*(cos(M_PI_4*(1-cos(M_PI/3*time_6)))-1);

               std::array<double, 16> current_pos = initial_pos_6;
//             current_pos[12] += dx;
//             current_pos[13] += dy;
               current_pos[14] += dz;

               franka::CartesianPose output = current_pos;

              if(time_6 >= finish_time_6)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                


                return output;
          };

          myRobot.control(cartesian_moving_Up);
//        // 7. Back to initial pos using simple motion generator
          myRobot.control(motiongenerator);


    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;

    }


  return 0;
}
