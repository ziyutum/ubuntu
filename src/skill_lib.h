#ifndef CHESSLIB_H
#define CHESSLIB_H

/* This library defines controllers that represent basic skills
 * of the panda robot. They are designed to be passed to
 * the robot.control() method inside a capability implementation.
 *
 * The here included controllers present simple movements.
 * They are meant to represent individual skills of the robot
 * in a relatively fine granularity, i.e. primitive skills.
 *
 * Combinations of these skills will result in capabilities.
 * The capabilities are implemented by the skills coded in this
 * libarary. The capability creation i.e. the arrangement of multiple
 * skills in a state machine is done in the programms that include
 * this libarary.
 *
 * Additionally operational variables that are shared among the basic skills
 * are included. The operational variables are set in the capability
 * implementations. They are defined in the Asset Administration Shell
 * of the robot.
 *
 * Author: Josua Hoefgen
 * Date: 05.08.2022
 */


/* Currently the following skills are implemetned:
 *  - CartesianMotionFunction
 *    Explanation: Moves the Endeffector by dX, dY, dZ
 *    Used several times in the capability GrabPieceAtXY
 *    (MoveAbovePiece, MoveDown, MoveUp)
 * - JointMotion      (TBD)
 * - TorqueController (TBD)
 * - Open Gripper (standard in gripper)
 * - Close Gripper (standard in gripper)
 */

// Includes for the robot control from libranka
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/gripper.h>
#include <franka/exception.h>

// For simplified motion to initial position
#include "examples_common.h"

// Includes for math and time
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <list>

// Includes for the TCP/IP socket connection
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>


// PROBLEM: [] capture list of the lambda needs to be empty otherwise
//          compile error, because vars are not declared.
// TRY: Define the necessary captures for the lambda functions
//      i.e. the skills also here in the header.
//      They become global vars that will be written at multiple
//      parts in the programm.


// TCP socket descriptor
int client_socket;

// Panda robot controller specific variables
double start_time = 0.0; // Each movement will start at t=0
double exec_time = 3.0; // Execution time for a movement
std::array<double, 16> current_initial_pose; // Inital pose as input for a cartesian motion
std::array<double, 7> current_joint_position; // Initial joint positions as input for a torque controle
using Vector7 = Eigen::Matrix<double, 7, 1>;
Vector7 p_gains; // gains for the torque control position contribution
Vector7 d_gains; // gains for the torque control velocity contribution


// DEFINITION OF OPERATIONAL VARIABLES FROM AAS
double GripperOpeningWidth = 0.04;
double GripperOpeningSpeed = 0.1;
double GripperClosingWidth = 0.03;
double GripperClosingSpeed = 0.05;
double GripperClosingForce = 1;
double GripperClosingForceResidual = 1e-5;

// Increments for the new position
double x_increment = 0;
double y_increment = 0;
double z_increment = 0;


/**
 * @brief: Skill-Description: A basic skill of the panda robot is to move the end
 *         effector(EE) to the cartesian position XYZ.
 **/
auto CartesianMotionFunction = [&current_initial_pose, &start_time, &exec_time,
        &x_increment, &y_increment, &z_increment, &client_socket] (const franka::RobotState& rstate
        , franka::Duration periode) -> franka::CartesianPose
    {
        start_time += periode.toSec();
        if(start_time == 0.0)
            current_initial_pose = rstate.O_T_EE_c;
        // Calculate the small movement increments
        double dx = -x_increment*(cos(M_PI_4*(1-cos(M_PI/exec_time*start_time)))-1);
        double dy = y_increment*(cos(M_PI_4*(1-cos(M_PI/exec_time*start_time)))-1);
        double dz = z_increment*(cos(M_PI_4*(1-cos(M_PI/exec_time*start_time)))-1);
        // Update next position the controller shall reach
        std::array<double, 16> current_pos = current_initial_pose;
        current_pos[12] += dx;
        current_pos[13] += dy;
        current_pos[14] += dz;

        franka::CartesianPose output = current_pos;

        if(start_time > exec_time){
            std::cout << "Finished moving in x,y,z" << std::endl;
            start_time = 0.0; // Reset start time for next call
            return  franka::MotionFinished(output);
        }

        // Send robot state via TCP/IP to the digital twin client app
        std::stringstream buffer;
        buffer << rstate;
        send(client_socket, buffer.str().c_str(), buffer.str().length(), MSG_DONTWAIT);

        return output;
    };


/**
 * @brief: Skill-Description: A basic skill of the panda robot is to control
 *         the torques in each joint.
 */
auto impedance_control_callback = [&current_joint_position, &p_gains, &d_gains, &client_socket](const franka::RobotState& rstate,
                                      franka::Duration periode) -> franka::Torques {

  // Conver the current joint positions to an eigen map
  Eigen::Map<const Vector7> initial_q(current_joint_position.data());

  // May include the corilis forces, but here not necessary
  //Eigen::Map<const Vector7> coriolis(model.coriolis(rstate).data());
  Eigen::Map<const Vector7> q(rstate.q.data());
  Eigen::Map<const Vector7> dq(rstate.dq.data());

  // Compute the neccessary torques to be applied to achieve net zero torque
  Vector7 t;
  t = 1*p_gains.cwiseProduct(initial_q-q) - 1*d_gains.cwiseProduct(dq);
  // Convert to an std:array
  std::array<double, 7> t_asArray{};
  Vector7::Map(&t_asArray[0]) = t;

  // Send robot state via TCP/IP to the digital twin client app
  std::stringstream buffer;
  buffer << rstate;
  send(client_socket, buffer.str().c_str(), buffer.str().length(), MSG_DONTWAIT);

  return t_asArray;
};

/**
 * @brief serializeOperationalData
 */
std::string serializeOperationalData(){

}


#endif // CHESSLIB_H
