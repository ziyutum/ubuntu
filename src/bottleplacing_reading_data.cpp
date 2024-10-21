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
        myRobot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//         // Set speedfactor
         double speedfactor = 0.1;

           franka::RobotState initial_state = myRobot.readOnce();
           
           std::cout << initial_state << std::endl;


          std::cout << "finish reading" << std::endl;



    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;

    }


  return 0;
}
