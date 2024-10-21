#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1],franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    // Setting the collision behavior appropriate for hands-on use
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    std::cout << "set collision sucessful"<<std::endl;

    //std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    

    // First move the robot to a suitable initial joint configuration
     std::array<double, 7> q_goal = {{0.414376,-0.602906,-0.51589,-2.36696,-0.305152,1.86541,0.772361}};


    MotionGenerator motion_generator(0.2, q_goal);
    std::cout << "move robot to initial position sucessful"<<std::endl;
    std::cout << "To reach initial position, Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 16> initial_pose;
    double time = 0.0;
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      constexpr double kRadius = 0.15;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = 0.27*0.325* std::sin(angle);
      // double delta_y = 0*-0.217* (std::sin(angle));
      // double delta_z = 0kRadius * (std::cos(angle) - 1);
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      // new_pose[13] += delta_y;
      // new_pose[14] += delta_z;

   
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion" << std::endl;
        std::cout << robot_state << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
     });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}