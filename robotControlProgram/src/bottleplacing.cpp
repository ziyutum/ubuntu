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
//         // Set an initial position
         int message_count = 0;


          std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
 //         gripper.move(0.1, 0.1);




         MotionGenerator motiongenerator(speedfactor, safepos);

//         std::cout << "Robot will move to initial pos..., after pressing enter" << std::endl;
//         std::cin.get();

            gripper.homing();
            myRobot.control(motiongenerator);
            gripper. move(0.04,0.1);
            

         std::cout << "Reached initial position" << std::endl;


//         //----------------------------------------------------------------------------------------------//
//         // Set up the TCP Socket, we use AF_INET (IPv4) even though we may communicate on the same host //
//         //----------------------------------------------------------------------------------------------//

            std::cout << "Setting up a socket connection with the DT client manager!" << std::endl;
            int server_fd = socket(AF_INET, SOCK_STREAM, 0);
            if(server_fd == 0)
                std::cout << "Error setting up server socket" << std::endl;

//         // Set some socket options
            int opt = 1;
            if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
            std::cout << "Error in Server Socket options" << std::endl;

//         // Define an address
            struct sockaddr_in address;
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
//         // htons converts from host to network byteorder
           address.sin_port = htons(8080); //  port from 8080 to 8081, otherwise no client socket can be created

//         // Bind the server socket to the port
            if(bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0){
            std::cout << "Error in binding the server." << std::endl;}

//         // But socket in listen mode
           int listen_res = listen(server_fd, 3);
//         // Accept an incoming connection request
            int addrlen = sizeof(address);
            int client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
            std::cout << "Server program, client_socket: "  << client_socket << std::endl;

//         // Send a welcome message
            char welcome_message[100] = "Welcome from FCI control program!";
            // send(client_socket, welcome_message, sizeof(welcome_message), MSG_CONFIRM);

            //  std::string test = "{\"q\": [0, -0.785241, 0, -2.35583 0, 1.57135, 0.785809,0.1,0.1]}\r\n";
            //     std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;
            //     //std::cout << test.length()<< std::endl; 
            //     std::cout << test<< std::endl;  
            //     send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);



//         //--------- END of TCP Socket Set-Up ------------------------------
          double time0 = 0.0;
          double finish_time0 = 6.0;

//         // at the initial position: echo_robot_state 192.168.3.100
//         // find the variable "O_T_EE"
//         // this is the initial_pos1 
          std::array<double, 16> initial_pos0; 
        

         auto cartesian_initial_motion_0 = [&initial_pos0, &time0, &finish_time0, &client_socket](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {
            
            time0 += periode.toSec();

            if(time0==0.0)
            {
                initial_pos0 = rstate.O_T_EE_c;
                // Send go for recoding to client app
                //send(client_socket, "GO", sizeof("GO"), MSG_DONTWAIT);
            }

            // Distance from the init position
            //////////////////////new
            //double dx = 0*(sin(M_PI_4*(1-cos(M_PI/5*time1))));
            //double dz = 0.265*(cos(M_PI_4*(1-cos(M_PI/5*time1)))-1);
            //double dz = -0.1*(cos(M_PI_4*(1-cos(M_PI/5*time1))));
              constexpr double kRadius = 0.4;
             double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time0));
             double dx = 0.0 * (cos(M_PI_4*(1-cos(M_PI/3*time0)))-1);
             double dy = 0* (cos(M_PI_4*(1-cos(M_PI/3*time0)))-1);
             double dz = 0 * (cos(M_PI_4*(1-cos(M_PI/3*time0)))-1);
            std::array<double, 16> current_pos = initial_pos0;
            current_pos[12] += dx;
            current_pos[13] += dy;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if(time0 >= finish_time0)
            {
                
                return  franka::MotionFinished(output);
            }
 //           
//             // previously it was std::string test
                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.0100);
                        if (test.length() < 100){
                            for (int j = 0; j<100-test.length(); j++){
                                test += std::to_string(0);
                                 std::cout<<test.length()<<std::endl;
                            }
                        }
                        
                    }
                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                
                //std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;
                //std::cout << test.length()<< std::endl; 
                //std::cout << test<< std::endl;  
                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);
//             //// end of new code
            return output;

         };
           //std::this_thread::sleep_for(std::chrono::seconds(10));
           myRobot.control(cartesian_initial_motion_0); 
//         // 1: Move down to pick something up
          std::cout << "First motion: Move down to the bottle position pick the bottle up"  << client_socket << std::endl;
          double time1 = 0.0;
          double finish_time1 = 3.0;

//         // at the initial position: echo_robot_state 192.168.3.100
//         // find the variable "O_T_EE"
//         // this is the initial_pos1 
          std::array<double, 16> initial_pos1; 
        

         auto cartesian_initial_motion_1 = [&initial_pos1, &time1, &finish_time1, &client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {
            std::cout <<rstate.O_T_EE_c[14] << std::endl;
            time1 += periode.toSec();

            if(time1==0.0)
            {
                initial_pos1 = rstate.O_T_EE_c;
                // Send go for recoding to client app
                send(client_socket, "GO", sizeof("GO"), MSG_DONTWAIT);
            }

            // Distance from the init position
            //////////////////////new
            //double dx = 0*(sin(M_PI_4*(1-cos(M_PI/5*time1))));
            //double dz = 0.265*(cos(M_PI_4*(1-cos(M_PI/5*time1)))-1);
            //double dz = -0.1*(cos(M_PI_4*(1-cos(M_PI/5*time1))));
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
 //           
//             // previously it was std::string test
                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.0100);
                        if (test.length() < 100){
                            for (int j = 0; j<100-test.length(); j++){
                                test += std::to_string(0);
                                 std::cout<<test.length()<<std::endl;
                            }
                        }
                        
                    }
                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;
                //std::cout << test.length()<< std::endl; 
                std::cout << test<< std::endl;  
                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);
//             //// end of new code
            return output;

         };
           std::this_thread::sleep_for(std::chrono::seconds(10));
           myRobot.control(cartesian_initial_motion_1);
           std::cout << "Reached position 2" << std::endl; 
//         // 2: Grasp
//         // First open the gripper
           std::cout << "2. Open gripper"  << client_socket << std::endl;
           //gripper.move(0.05, 0.01);
           gripper.grasp(0.01, 0.02, 70);
           std::cout << " Finished grasping"  << std::endl;


        
        
//         // 3. Move up
        std::cout << "3. Move up"  << client_socket << std::endl;
         double time_2 = 0.0;
         double finish_time_2 = 3.0;
         std::array<double, 16> initial_pos_2;

         auto cartesian_moving_slightlyUp = [&initial_pos_2, &time_2, &finish_time_2,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
        {

            time_2 += periode.toSec();


            if(time_2==0.0)
            {
                initial_pos_2 = rstate.O_T_EE_c;
            }


//             double dx = -0.657*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1); //-1 to move back and to init again
//             double dy = -0.0215*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1);
                double dz = -0.105*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1);

               std::array<double, 16> current_pos = initial_pos_2;
//             current_pos[12] += dx;
//             current_pos[13] += dy;
               current_pos[14] += dz;

               franka::CartesianPose output = current_pos;

              if(time_2 >= finish_time_2)
                {
                    std::cout << "End control loop" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                std::cout << "t:" << time_2 << "  periode: " << periode.toSec() << " z-Koord.: " << rstate.O_T_EE_c[14] << std::endl;


                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.020);
                        if (test.length() < 100){
                            for (int i = 0; i<100-test.length(); i++){
                                test += std::to_string(0);
                            }
                        }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;


                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);


                return output;
          };
           std::this_thread::sleep_for(std::chrono::seconds(2));

          myRobot.control(cartesian_moving_slightlyUp);
 
 //         4. Move around
             std::cout << "4. Move around"  << std::endl;
           double time_3 = 0.0;
           double finish_time_3 = 3.0;
           std::array<double, 16> initial_pos_3; 

            auto cartesian_moving_around = [&initial_pos_3, &time_3, &finish_time_3,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
            {

                time_3 += periode.toSec();


                if(time_3==0.0)
                {
                    initial_pos_3 = rstate.O_T_EE_c;
                }



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
                    std::cout << "End moving" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                std::cout << "t:" << time_3 << "  periode: " << periode.toSec() << " z-Koord.: " << rstate.O_T_EE_c[14] << std::endl;


                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.020);
                        if (test.length() < 100){
                            for (int i = 0; i<100-test.length(); i++){
                                test += std::to_string(0);
                            }
                        }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;


                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);


                return output;
                
            };
           std::this_thread::sleep_for(std::chrono::seconds(2));
            myRobot.control(cartesian_moving_around);


//    //4. 2 + Move around
             std::cout << "4.2 Move around"  << std::endl;
           double time_32 = 0.0;
           double finish_time_32 = 4.0;
           std::array<double, 16> initial_pos_32; 

            auto cartesian_moving_around2 = [&initial_pos_32, &time_32, &finish_time_32,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
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

                 std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.020);
                        if (test.length() < 100){
                            for (int i = 0; i<100-test.length(); i++){
                                test += std::to_string(0);
                            }
                        }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;


                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);




                return output;
                
            };
           std::this_thread::sleep_for(std::chrono::seconds(2));

            myRobot.control(cartesian_moving_around2);

//    //4. 3 + Move around
             std::cout << "4.3 Move around"  << std::endl;
           double time_33 = 0.0;
           double finish_time_33 = 4.0;
           std::array<double, 16> initial_pos_33; 

            auto cartesian_moving_around3 = [&initial_pos_33, &time_33, &finish_time_33,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
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

                 std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.020);
                        if (test.length() < 100){
                            for (int i = 0; i<100-test.length(); i++){
                                test += std::to_string(0);
                            }
                        }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;


                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);


             


                return output;
                
            };
           std::this_thread::sleep_for(std::chrono::seconds(2));

            myRobot.control(cartesian_moving_around3);


//         // 5: place object
         std::cout << "5. Place the bottle"  << std::endl;
           double time_4 = 0.0;
           double finish_time_4 = 3.0;
          std::array<double, 16> initial_pos_4;

        auto cartesian_moving_slightlyDown = [&initial_pos_4, &time_4, &finish_time_4,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose{

           time_4 += periode.toSec();


           if(time_4==0.0)
           {
               initial_pos_4 = rstate.O_T_EE_c;
           }



               double dz = 0.15*(cos(M_PI_4*(1-cos(M_PI/3*time_4)))-1);

              std::array<double, 16> current_pos = initial_pos_4;
//            //current_pos[12] += dx;
              current_pos[14] += dz;

             franka::CartesianPose output = current_pos;

                if(time_4 >= finish_time_4)
                {
                    std::cout << "End placing" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                std::cout << "t:" << time_4 << "  periode: " << periode.toSec() << " z-Koord.: " << rstate.O_T_EE_c[14] << std::endl;

                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.02);
                        if (test.length() < 100){
                                for (int j = 0; j<100-test.length(); j++){
                                    test += std::to_string(0);
                                }
                            }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;



                send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);

                return output;
           };
           std::this_thread::sleep_for(std::chrono::seconds(2));

          myRobot.control(cartesian_moving_slightlyDown);

//        // 6. Open the gripper
          gripper.move(0.03, 0.03);
          gripper.stop();


//         // 7. Move up
         std::cout << "7. Move up"  << std::endl;
         double time_6= 0.0;
         double finish_time_6 = 3.0;
         std::array<double, 16> initial_pos_6;

         auto cartesian_moving_Up = [&initial_pos_6, &time_6, &finish_time_6,&client_socket,&message_count](const franka::RobotState& rstate, franka::Duration periode) -> franka::CartesianPose
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
                    std::cout << "Leaving the station" << std::endl;
                    std::cout << rstate << std::endl;
                    return  franka::MotionFinished(output);
                }

                std::cout << "t:" << time_6 << "  periode: " << periode.toSec() << " z-Koord.: " << rstate.O_T_EE_c[14] << std::endl;


                std::string test = "{\"q\": [";
                for (int i = 0;i<9;i++){
                    if (i<7){
                        std::string q = std::to_string(rstate.q[i]);
                        test += q;
                    }
                    else {
                        test += std::to_string(0.030);
                        if (test.length() < 100){
                            for (int i = 0; i<100-test.length(); i++){
                                test += std::to_string(0);
                            }
                        }
                    }


                    if (i<8){
                        test += ",";
                    }

                }
                test += "]}\r\n";
                message_count++;

                std::cout << "send message to client:"<< test.c_str()<<test.length() << std::endl;


//             // Send robot state over TCP/IP to the Client App
               send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);


                return output;
          };
           std::this_thread::sleep_for(std::chrono::seconds(2));

          myRobot.control(cartesian_moving_Up);
          
//        // 6. Back to initial position
       std::cout << "8. To Initial"  << std::endl;
        myRobot.control(motiongenerator);
        std::cout << message_count << std::endl;
        //  std::string test1 = "{\"q\": [0, -0.785241, 0, -2.35583 0, 1.57135, 0.785809,0.1,0.1]}\r\n";
        // std::cout << "send message to client:"<< test1.c_str()<<test1.length() << std::endl;
        //         //std::cout << test.length()<< std::endl; 
        // std::cout << test1<< std::endl;  
        // send(client_socket, test1.c_str(), test1.length(), MSG_DONTWAIT);


    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;

    }


  return 0;
}
