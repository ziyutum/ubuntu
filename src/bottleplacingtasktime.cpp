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
#include <fstream>

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
          double finish_time1 = 9.0;

//         // at the initial position: echo_robot_state 192.168.3.100
//         // find the variable "O_T_EE"
//         // this is the initial_pos1 
          std::array<double, 16> initial_pos1; 
            // Open file to save callback execution times
        std::ofstream time_file("callback_times.txt");
        if (!time_file) {
            std::cerr << "Error opening file to save callback execution times." << std::endl;
            return -1;
        }
        time_file << "Callback Execution Time (μs)" << std::endl;

        

                auto cartesian_initial_motion_1 = [&initial_pos1, &time1, &finish_time1, &time_file](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {
            static auto last_time = std::chrono::high_resolution_clock::now();
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = current_time - last_time;
            last_time = current_time;
            time_file << elapsed.count() * 1e6 << std::endl; // Save time to file
            time_file.flush();
        

            std::cout << "Callback execution time: " << elapsed.count() * 1e6 << " μs" << std::endl;
            // static auto start = std::chrono::high_resolution_clock::now(); // 
            // auto current_time = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = current_time - start_time;
            
            // std::cout << "Callback execution time: " << elapsed.count() * 1e6 << " μs" << std::endl;
            
            //std::cout << rstate.O_T_EE_c[14] << std::endl;
            time1 += periode.toSec();

            if (time1 == 0.0) {
                initial_pos1 = rstate.O_T_EE_c;
            }

            constexpr double kRadius = 0.4;
            double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time1));
            double dx = 0.08 * (cos(M_PI_4 * (1 - cos(M_PI / 3 * time1))) - 1);
            double dy = 0.26 * (cos(M_PI_4 * (1 - cos(M_PI / 3 * time1))) - 1);
            double dz = kRadius * (cos(M_PI_4 * (1 - cos(M_PI / 3 * time1))) - 1);
            std::array<double, 16> current_pos = initial_pos1;
            current_pos[12] += dx;
            current_pos[13] += dy;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if (time1 >= finish_time1) {
                std::cout << "End moving to the position" << std::endl;
                std::cout << rstate << std::endl;
                return franka::MotionFinished(output);
            }
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // std::cout << "Callback execution time: " << elapsed.count() * 1e6 << " μs" << std::endl;

            return output;
        };
           
        // auto start = std::chrono::high_resolution_clock::now();
           myRobot.control(cartesian_initial_motion_1);
        //    auto end = std::chrono::high_resolution_clock::now();
        //     std::chrono::duration<double> elapsed = end - start;
        //     std::cout << "User code execution time: " << elapsed.count() * 1e6 << " μs" << std::endl;

         time_file.close();

           std::cout << "Reached position 2" << std::endl; 


           
//         // 2: Grasp
//         // First open the gripper
           std::cout << "2. Open gripper"  << std::endl;
           
           gripper.grasp(0.01, 0.02, 70);
           std::cout << " Finished grasping"  << std::endl;

        
        
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
