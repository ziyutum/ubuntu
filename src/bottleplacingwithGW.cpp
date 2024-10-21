// the gripper state are read as opening width, not self defined.
 #include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <string>
#include <ctime>
#include <iomanip>

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

// Function to move the robot in Cartesian coordinates and send TCP/IP messages
void MoveCartesian(franka::Robot& robot, std::vector<double> targetCoordinates, double movementTime, int client_socket, double GW) {
    std::array<double, 16> initial_pose;
    double time = 0.0;
    double total_time = movementTime;
    int update_count = 0 ; // Feng Xu 12.07.2024 initialize a counter

    auto cartesian_motion = [&time, &total_time, &initial_pose, &targetCoordinates, &client_socket, &GW, &update_count](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        update_count++; // Feng Xu 12.07.2024 update for every frequence

        if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
        }

        double dx = targetCoordinates[0] * (cos(M_PI_4 * (1 - cos(M_PI / total_time * time))) - 1);
        double dy = targetCoordinates[1] * (cos(M_PI_4 * (1 - cos(M_PI / total_time * time))) - 1);
        double dz = targetCoordinates[2] * (cos(M_PI_4 * (1 - cos(M_PI / total_time * time))) - 1);

        std::array<double, 16> new_pose = initial_pose;
        new_pose[12] += dx;
        new_pose[13] += dy;
        new_pose[14] += dz;

        //std::string test = "{\"q\": [";
        std::string test = "[";
        for (int i = 0; i < 9; i++) {
            if (i < 7) {
                std::string q = std::to_string(robot_state.q[i]);
                test += q;
            } else {
                test += std::to_string(GW); // 
                if (test.length() < 100) {
                    for (int j = 0; j < 100 - test.length(); j++) {
                        test += std::to_string(0);
                    }
                }
            }
            if (i < 8) {
                test += ",";
            }
        }
        //test += "]}\r\n";
        test += "]\r\n";
        
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为time_t格式
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        // 转换为tm结构
        std::tm* now_tm = std::localtime(&now_time_t);
        // 取当前时间点的毫秒部分
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        // 打印当前时间和毫秒
        std::cout << "Current local time: " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S")
                << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << std::endl;
    
        std::cout << "send message to client:" << test.c_str() << test.length() << std::endl;
        send(client_socket, test.c_str(), test.length(), MSG_DONTWAIT);

        if (time >= total_time) {
            return franka::MotionFinished(new_pose);
        }
        return new_pose;
    };

    robot.control(cartesian_motion);
}

// this code is only used for automatica 
int main(int argc, char** argv) {

    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try
    {
        // Create robot instance
        franka::Robot myRobot(argv[1], franka::RealtimeConfig::kIgnore);

        // Create the gripper instance
        franka::Gripper gripper(argv[1]);

        // Setting the collision behavior appropriate for hands-on use
        myRobot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // Set speedfactor
        double speedfactor = 0.3;
        // Set an initial position
        std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        gripper.move(0.03, 0.01);

        MotionGenerator motiongenerator(speedfactor, safepos);

        // std::cout << "Robot will move to initial pos..., after pressing enter" << std::endl;
        // std::cin.get();

        myRobot.control(motiongenerator);
        gripper.homing();

        std::cout << "Reached initial position" << std::endl;
        std::cout << "Continue moving down" << std::endl;

        //----------------------------------------------------------------------------------------------//
        // Set up the TCP Socket, we use AF_INET (IPv4) even though we may communicate on the same host //
        //----------------------------------------------------------------------------------------------//

        std::cout << "Setting up a socket connection with the DT client manager!" << std::endl;
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == 0)
            std::cout << "Error setting up server socket" << std::endl;

        // Set some socket options
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
            std::cout << "Error in Server Socket options" << std::endl;

        // Define an address
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8080); // Hou: here i changed the port from 8080 to 8081, otherwise no client socket can be created

        // Bind the server socket to the port
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cout << "Error in binding the server." << std::endl;
        }

        // Put socket in listen mode
        int listen_res = listen(server_fd, 3);
        // Accept an incoming connection request
        int addrlen = sizeof(address);
        int client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        std::cout << "Server program, client_socket: " << client_socket << std::endl;

        // Send a welcome message
        //char welcome_message[100] = "Welcome from FCI control program!";
        //send(client_socket, welcome_message, sizeof(welcome_message), MSG_CONFIRM);

        // Feng Xu 08.07.2024
        // Declare gripper State andGW here
        franka::GripperState state;
        double GW;

        // 1: Move down to pick something up
        std::cout << "First motion: Move down to pick sth up" << client_socket << std::endl;
        std::vector<double> targetCoordinates1 = {0.08, 0.26, 0.4};
        gripper.move(0.04, 0.01);
        double movementTime1 = 3.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates1, movementTime1, client_socket,GW);

        // 2: Grasp
        // First close the gripper
        std::cout << "2. Close gripper" << client_socket << std::endl;
        // gripper.move(0.03, 0.01);
        gripper.grasp(0.01, 0.02, 70);

        // 3. Moveup
        std::cout << "3. Move up" << client_socket << std::endl;
        std::vector<double> targetCoordinates2 = {0, -0, -0.105};
        double movementTime2 = 3.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates2, movementTime2, client_socket,GW);


//-----------------------------------------------------------------------------------------
        std::cout << "41. Move around" << client_socket << std::endl;
        std::vector<double> targetCoordinates41 = {-0.08, -0.26, -0.295};
        double movementTime41 = 3.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates41, movementTime41, client_socket,GW);

        std::cout << "42. Move around" << client_socket << std::endl;
        std::vector<double> targetCoordinates42 = {0.445, -0.325, 0.1};
        double movementTime42 = 4.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates42, movementTime42, client_socket,GW);


        std::cout << "43. Move around" << client_socket << std::endl;
        std::vector<double> targetCoordinates43 = {0.445, 0.1, -0.005};
        double movementTime43 = 4.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates43, movementTime43, client_socket,GW);






        // 4: place object
        std::vector<double> targetCoordinates4 = {0.0, 0.0, 0.2};
        double movementTime4 = 3.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates4, movementTime4, client_socket, GW);

        // 5. Open the gripper
        gripper.move(0.03, 0.03);
        gripper.stop();

        // 6. Back to initial pos using simple motion generator
        std::vector<double> targetCoordinates5 = {0.657, 0.0215, -0.46};
        double movementTime5 = 5.0;
        
        // Feng Xu 08.07.2024
        state = gripper.readOnce();
        GW = state.width;

        MoveCartesian(myRobot, targetCoordinates5, movementTime5, client_socket, GW);

    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
