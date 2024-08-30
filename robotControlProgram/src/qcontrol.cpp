#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"
int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1],franka::RealtimeConfig::kIgnore);
    franka::Gripper gripper(argv[1]);
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0.202,-0.1445,-0.282566,-2.47408,0.0419634,2.45,0.0683091}}; 
    MotionGenerator motion_generator(0.3, q_goal);
    //to get to the initial position

    robot.control(motion_generator);
    gripper.move(0.03,0.05);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


// No.1 rotation
    // std::cout  << "Press Enter to move" << std::endl;
    // std::cin.ignore();
    std::array<double, 7> initial_position;

    double time = 0.0;
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 16.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0]- 1.4*delta_angle, initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4] , initial_position[5],
                                        initial_position[6] }};
      if (time >= 2.5) {
        std::cout << std::endl << "Finished the first motion" << std::endl;
        std::cout  << robot_state  << std::endl;
        return franka::MotionFinished(output);
        
      }
      return output;
    });


// No.2 Move downwards
    // std::cout  << "Press Enter to move" << std::endl;
    // std::cin.ignore();
std::array<double, 7> initial_position2;
    double time_2 = 0.0;
    robot.control([&initial_position2, &time_2](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time_2 += period.toSec();
      if (time_2== 0.0) {
        initial_position2= robot_state.q_d;
      }
      double delta_angle = M_PI / 16.0 * (1 - std::cos(M_PI / 2.5 * time_2));
      franka::JointPositions output = {{initial_position2[0], initial_position2[1]+1.08*delta_angle,
                                        initial_position2[2], initial_position2[3],
                                        initial_position2[4] , initial_position2[5]+0.8*delta_angle ,
                                        initial_position2[6]}};
      if (time_2 >= 2.5) {
        std::cout << std::endl << "Finished the second motion" << std::endl;
        std::cout  << robot_state  << std::endl;
        return franka::MotionFinished(output);
      }
      return output;

    });



           std::cout << "2. Open gripper"  << std::endl;
           gripper.move(0.03, 0.01);
           gripper.grasp(0.01, 0.01, 70);
           std::cout << " Finished grasping"  << std::endl;

// No.3 Move upwards
    // std::cout  << "Press Enter to move up" << std::endl;
    // std::cin.ignore();
    std::array<double, 7> initial_position_3;
    double time_3 = 0.0;
    robot.control([&initial_position_3, &time_3](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time_3 += period.toSec();
      if (time_3 == 0.0) {
        initial_position_3= robot_state.q_d;
      }
      double delta_angle = M_PI / 16.0 * (1 - std::cos(M_PI / 2.5 * time_3));
      franka::JointPositions output = {{initial_position_3[0], initial_position_3[1]-1.5*delta_angle,
                                        initial_position_3[2], initial_position_3[3],
                                        initial_position_3[4] , initial_position_3[5]-2.7*delta_angle ,
                                        initial_position_3[6]}};
      if (time_3 >= 2.5) {
        std::cout << std::endl << "Finished the second motion" << std::endl;
        std::cout  << robot_state  << std::endl;
        return franka::MotionFinished(output);
      }
      return output;

    });


// No.4 Move around
    // std::cout  << "Press Enter to move up" << std::endl;
    // std::cin.ignore();
    std::array<double, 7> initial_position_4;
    double time_4 = 0.0;
    robot.control([&initial_position_4, &time_4](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time_4 += period.toSec();
      if (time_4== 0.0) {
        initial_position_4= robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 5 * time_4));
      franka::JointPositions output = {{initial_position_4[0]+1.3*delta_angle , initial_position_4[1],
                                        initial_position_4[2], initial_position_4[3],
                                        initial_position_4[4] , initial_position_4[5],
                                        initial_position_4[6]}};
      if (time_4 >= 5) {
        std::cout << std::endl << "Finished the 4th motion" << std::endl;
        std::cout  << robot_state  << std::endl;
        return franka::MotionFinished(output);
      }
      return output;

    });


























































































  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}