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

// Hardware specific constants
#define BOARD_FIELD_SPACING 0.045 // The distance between the center of chess board fields


// Chess game specific data types
struct chess_move
{
    char piece_type; // The piece, necessary to calculate gripper height and width
    std::array<int,2> start_position;  // Starting and target position on the board
    std::array<int,2> target_position; // defined as X,Y coordinates.
    bool isRemove; // Indicates if in the next move the piece that is removed will be hit
};

// Forward declarations
bool move_to_init(franka::Robot& rob);
bool grab_or_place_pieceT_atXY(franka::Robot& rob, franka::Gripper& grip, char piece_type, std::array<int,2> board_index, bool grab, int client_socket);
bool remove_pieceT_atXY(franka::Robot& rob, franka::Gripper& grip, char piece_type, std::array<int,2> board_index, int client_socket);
std::list<chess_move> generate_move_list();

int main(int argc, char** argv)
{
    // Check that ip adress of fci is given
    if(argc != 2)
    {
        std::cerr << "FCI I.P. not specified only argv[0]: " << argv[0] << std::endl;
    }

    // Wrap whole control structure in try block to safely handle any occuring exceptions
    try {
        // Create robot instance
        franka::Robot robot(argv[1]);
        // Create the gripper instance
        franka::Gripper gripper(argv[1]);
        // Set an appropriate collision behavior
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        //  Move to init and home gripper
        std::cout << "Robot will move to initial pos..." << std::endl;
        std::cin.get();
        bool bMovedToInit;
        move_to_init(robot);
        gripper.move(0.03, 0.1);
        if(bMovedToInit)
            std::cout << "Reached initial position" << std::endl;

        // Print out current robot state
        franka::RobotState robot_state_init = robot.readOnce();
        std::cout << robot_state_init << std::endl;
        std::array<double, 16> end_effector_pos_init = robot_state_init.O_T_EE;
        std::cout << "Robot End-Effector position: x=" << end_effector_pos_init[12]
                  << " y=" << end_effector_pos_init[13] << " z=" << end_effector_pos_init[14] << std::endl;


        //----------------------------------------------------------------------------------------------//
        // Set up the TCP Socket, we use AF_INET (IPv4) even though we may communicate on the same host //
        //----------------------------------------------------------------------------------------------//

        std::cout << "Setting up a socket connection with the DT client manager!" << std::endl;
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if(server_fd == 0)
            std::cout << "Error setting up server socket" << std::endl;

        // Set some socket options
        int opt = 1;
        if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
           std::cout << "Error in Server Socket options" << std::endl;

        // Define an adress
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        // htons converts from host to network byteorder
        address.sin_port = htons(8080);

        // Bind the server socket to the port
        if(bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0){
           std::cout << "Error in binding the server." << std::endl;}

        // Put socket in listen mode
        int listen_res = listen(server_fd, 3);

        // Accept an incoming connection request
        int addrlen = sizeof(address);
        int client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        std::cout << "Server program, client_socket: "  << client_socket << std::endl;

        // Send a welcome message
        char welcome_message[100] = "Welcome from FCI control program!";
        send(client_socket, welcome_message, sizeof(welcome_message), MSG_CONFIRM);


        //--------- END of TCP Socket Set-Up ------------------------------


//        // TEST: Move a piece across the whole board
//        bool bMovedTo_0_0;
//        std::array<int,2> sample_index {0,0};
//        gripper.move(0.04, 1);
//        std::array<int,2> target_index;
//        for(int x=6; x<=7; x++)
//        {
//            for(int y=0; y <=7; y++)
//            {
//                sample_index = {x,y};
//                std::cout << "Moving to grab at (" << x << "," << y << ") " << std::endl;
//                grab_or_place_pieceT_atXY(robot, gripper, 'S', sample_index, true);
//                move_to_init(robot);
//                // Now place in next field
//                if(x == 7 && y < 7)
//                    target_index = {0, y+1};
//                else if(x < 7 && y == 7)
//                    target_index = {x+1,0};
//                else if(x==7 && y == 7)
//                    target_index = {0,0};
//                else
//                    target_index = {x,y+1};
//                std::cout << "Moving to place at (" << target_index[0] << "," << target_index[1] << ") " << std::endl;
//                grab_or_place_pieceT_atXY(robot, gripper, 'S', target_index, false);
//                move_to_init(robot);
//            }
//        }

//        // TEST: Remove a piece from an position X,Y
//        std::array<int,2> pos{5,4};
//        remove_pieceT_atXY(robot, gripper, 'S', pos);

        // TEST: A few moves from a real game
        std::list<chess_move> all_moves = generate_move_list();

        for(const auto& move : all_moves)
        {
            std::cout << "Next move: \nPiece Type:" << move.piece_type << "\nFrom: (" << move.start_position[0] << ","
                      << move.start_position[1] << ")\nTo: (" << move.target_position[0] << "," << move.target_position[1] << ")" << std::endl;

            // For now the move list will include the removes specifically
            if(not move.isRemove )
            {
                // Grab the next piece
                grab_or_place_pieceT_atXY(robot, gripper, move.piece_type, move.start_position, true, client_socket);
                move_to_init(robot);
                grab_or_place_pieceT_atXY(robot, gripper, move.piece_type, move.target_position, false, client_socket);
                move_to_init(robot);
            }
            else if(move.isRemove)
            {
                remove_pieceT_atXY(robot, gripper, move.piece_type, move.start_position, client_socket);
            }
        }

        // Send END command to Client App, to let it know that it can close its socket connection
        char end_message[100] = "END";
        send(client_socket, end_message, sizeof(end_message), MSG_CONFIRM);

        std::cout << "Finished all fields" << std::endl;
        move_to_init(robot);
    }
    catch (franka::Exception franka_ex)
    {
        std::cout << franka_ex.what() << std::endl;
    }
    return 0;
}


bool move_to_init(franka::Robot& rob){
    // Set a speedfactor
    double speedfactor = 0.3;
    // Set initial position by specifying joint angles
    std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // Get an already implemented motion generator
    MotionGenerator motiongenerator(speedfactor, safepos);
    // Call the robot controll with the motion generator to execute
    try
    {
        rob.control(motiongenerator);
    }
    catch(franka::Exception franka_ex)
    {
        std::cout << franka_ex.what() << std::endl;
    }

    return true;
}

bool grab_or_place_pieceT_atXY(franka::Robot& rob, franka::Gripper& grip, char piece_type, std::array<int,2> board_index, bool grab, int client_socket) {//char piece_type, std::array<int,2>& board_index){

    /*  Graps a piece at a board position X-Y.
     *  Different heights depending on piece type t.
     *  Moves to a waiting position to execute next command.
     */

    // Define time in which the procedure should be finished
    double time = 0.0; //! [s]
    double finish_time = 3.0; //! [s]

    std::array<double, 16> current_initial_pos;

    // Define the control callback function
    // We use a cartesion motion
    auto cartesian_motion_to_XY = [&current_initial_pos, &time, &finish_time, &piece_type, &board_index, &client_socket] (const franka::RobotState& rstate,
            franka::Duration periode) -> franka::CartesianPose
        {
            time += periode.toSec();

            if(time==0.0)
                current_initial_pos = rstate.O_T_EE_c;

            // Determine the necessary gripper height based on chess piece type

            // Refactor: Logic to compute will be grouped in capability
            double x_distance = -0.08-BOARD_FIELD_SPACING*board_index[0];
            double y_distance = -0.17+BOARD_FIELD_SPACING*board_index[1];
            double z_distance = 0.1;


            double dx = x_distance*(cos(M_PI_4*(1-cos(M_PI/3*time)))-1);
            double dy = y_distance*(cos(M_PI_4*(1-cos(M_PI/3*time)))-1);
            double dz = z_distance*(cos(M_PI_4*(1-cos(M_PI/3*time)))-1);

            std::array<double, 16> current_pos = current_initial_pos;
            current_pos[12] += dx;
            current_pos[13] += dy;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;


            if(time > finish_time)
            {
                std::cout << "Finished moving in x,y to " << board_index[0] << ", " << board_index[1] << std::endl;
                return  franka::MotionFinished(output);
            }

            // Test wise get a component of the robot as string
            std::string test = std::to_string(rstate.O_T_EE[14]);

            // Send robot state over TCP/IP to the Client App
            send(client_socket, test.c_str(), sizeof(test.c_str()), MSG_DONTWAIT);
            return output;
        };

    // Invoke robot controller
    rob.control(cartesian_motion_to_XY);


    // Define time in which the procedure should be finished
    double time_2 = 0.0; //! [s]
    double finish_time_2 = 3.0; //! [s]

    std::array<double, 16> current_initial_pos_2;

    // Define the control callback function
    // We use a cartesion motion
    auto cartesian_motion_down = [&current_initial_pos_2, &time_2, &finish_time_2, &piece_type, &board_index] (const franka::RobotState& rstate,
            franka::Duration periode) -> franka::CartesianPose
        {
            time_2 += periode.toSec();

            if(time_2==0.0)
                current_initial_pos_2 = rstate.O_T_EE_c;

            // Determine the necessary gripper height based on chess piece type

            // ATTENTION: ALREADY MOVED 0.1 DOWN

            double z_distance = 0.1;
            //std::cout << piece_type << (piece_type == 'Q') << std::endl;
            if(piece_type == 'P')
                z_distance = 0.36;
            else if(piece_type == 'Q' || piece_type=='K')
                z_distance = 0.335;
            else if(piece_type=='L' || piece_type=='T')
                z_distance = 0.347;
            else if(piece_type == 'S')
                z_distance = 0.34;


            double dz = z_distance*(cos(M_PI_4*(1-cos(M_PI/3*time_2)))-1);

            std::array<double, 16> current_pos = current_initial_pos_2;
            current_pos[14] += dz;
            franka::CartesianPose output = current_pos;

            if(time_2 > finish_time_2)
            {
                std::cout << "Finished moving down for piece T:"<< piece_type  << std::endl;
                return  franka::MotionFinished(output);
            }

            return output;
        };

    rob.control(cartesian_motion_down);

    // Control the gripper
    if(grab)
    {
        if(piece_type == 'T' || piece_type == 'K' || piece_type == 'Q' || piece_type == 'L')
            grip.grasp(0.02, 0.04, 1);
        else
            grip.grasp(0.01, 0.04, 1);
    }
    else
    {
        grip.move(0.03, 0.05);
    }
    // Move up again, only then to init
    double time_3 = 0.0; //! [s]
    double finish_time_3 = 3.0; //! [s]

    std::array<double, 16> current_initial_pos_3;

    auto cartesian_motion_up = [&current_initial_pos_3, &time_3, &finish_time_3, &piece_type, &board_index] (const franka::RobotState& rstate,
            franka::Duration periode) -> franka::CartesianPose
        {
            time_3 += periode.toSec();

            if(time_3==0.0)
                current_initial_pos_3 = rstate.O_T_EE_c;

            // Determine the necessary gripper height based on chess piece type

            // ATTENTION: ALREADY MOVED 0.1 DOWN

            double z_distance = -0.2;

            double dz = z_distance*(cos(M_PI_4*(1-cos(M_PI/3*time_3)))-1);

            std::array<double, 16> current_pos = current_initial_pos_3;

            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if(time_3 > finish_time_3)
            {
                std::cout << "Finished moving up again." << std::endl;
                return  franka::MotionFinished(output);
            }
            return output;
        };

    rob.control(cartesian_motion_up);

    return true;
}

bool remove_pieceT_atXY(franka::Robot& rob, franka::Gripper& grip, char piece_type, std::array<int,2> board_index, int client_socket)
{
    /* Removes a figure from the field by first grabbing it
     * and then releasing it over a predefined position.
     */

    // First grab the piece
    grab_or_place_pieceT_atXY(rob, grip, piece_type, board_index, true, client_socket);

    // Move just straight up first to avoid pushing over other pieces
    double time = 0.0; //! [s]
    double finish_time = 2.0; //! [s]
    std::array<double, 16> initial_pos;

    auto cartesian_motion_up = [&initial_pos, &time, &finish_time] (const franka::RobotState& rstate,
            franka::Duration periode) -> franka::CartesianPose
        {
            time += periode.toSec();

            if(time==0.0)
                initial_pos = rstate.O_T_EE_c;

            double z_distance = -0.3;
            double dz = z_distance*(cos(M_PI_4*(1-cos(M_PI/2*time)))-1);

            std::array<double, 16> current_pos = initial_pos;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if(time > finish_time)
            {
                std::cout << "Finished moving up (pre-remove)" << std::endl;
                return  franka::MotionFinished(output);
            }

            return output;
        };

    rob.control(cartesian_motion_up);

    // Now move to release pos
    double time_2 = 0.0; //! [s]
    double finish_time_2 = 5.0; //! [s]
    std::array<double, 16> initial_pos_2;

    auto cartesian_motion_to_remove = [&initial_pos_2, &time_2, &finish_time_2] (const franka::RobotState& rstate,
            franka::Duration periode) -> franka::CartesianPose
        {
            time_2 += periode.toSec();

            if(time_2==0.0)
                initial_pos_2 = rstate.O_T_EE_c;

            double x_distance = 0.4;
            double y_distance = -0.20;
            double z_distance = 0.2;
            double dx = x_distance*(cos(M_PI_4*(1-cos(M_PI/5*time_2)))-1);
            double dy = y_distance*(cos(M_PI_4*(1-cos(M_PI/5*time_2)))-1);
            double dz = z_distance*(cos(M_PI_4*(1-cos(M_PI/5*time_2)))-1);

            std::array<double, 16> current_pos = initial_pos_2;
            current_pos[12] += dx;
            current_pos[13] += dy;
            current_pos[14] += dz;

            franka::CartesianPose output = current_pos;

            if(time_2 > finish_time_2)
            {
                std::cout << "Finished moving up (pre-remove)" << std::endl;
                return  franka::MotionFinished(output);
            }

            return output;
        };

    rob.control(cartesian_motion_to_remove);

    // Open the gripper
    grip.move(0.04, 0.1);
    // return to init
    move_to_init(rob);


    return true;
}

std::list<chess_move> generate_move_list()
{
    /*  LAYOUT OF OUR CHESS BOARD
     *
     *      0   1   2   3   4   5   6   7 -> X
     *   0 wT  wP
     *   1 wS  wP
     *   2 wL  wP
     *   3 wK
     *   4 wQ
     *   5
     *   6
     *   7
     *   |
     *   \/
     *   Y
     *
     *  Therefore: A->0, B->1, C->2, D->3, E->4, F->5, G->6, h->7
     */

    std::list<chess_move> move_list = { {'S', {0,6}, {2,5}, false},
                                        {'S', {7,6}, {5,5}, false},
                                        {'P', {1,2}, {3,2}, false},
                                        {'P', {6,6}, {5,6}, false},
                                        {'S', {0,1}, {2,2}, false},
                                        {'L', {7,5}, {6,6}, false},
//                                        {'P', {1,3}, {3,3}, false},
//                                        {'T', {7,7}, {4,8}, false},
//                                        {'K', {7,4}, {7,6}, false},
//                                        {'T', {4,8}, {7,5}, false},
//                                        {'L', {0,2}, {3,5}, false},
//                                        {'P', {6,3}, {4,3}, false},
//                                        {'Q', {0,3}, {2,1}, false},
//                                        {'P', {3,2}, {-1,-1}, true},
//                                        //{'P', {4,3}, {3,2}, false},
                                        //{'P', {3,2}, {-1,-1}, true},
                                        //{'Q', {2,1}, {3,2}, false},
                                        //{'P', {6,2}, {5,2}, false},
                                        //{'P', {1,4}, {3,4}, false},
                                        //{'S', {7,1}, {6,3}, false},
                                        //{'T', {0,0}, {0,3}, false},
                                        //{'S', {6,3}, {5,1}, false},
                                        //{'Q', {3,2}, {4,2}, false},
                                        //{'L', {7,2}, {3,6}, false},
                                        //{'L', {3,5}, {4,6}, false},
                                        //{'S', {5,1}, {3,0}, false},
                                        //{'Q', {4,2}, {2,0}, false},
                                        //{'S', {2,2}, {-1,-1}, true},
                                        //{'S', {3,0}, {2,2}, false},
                                        //{'S', {2,2}, {-1,-1}, true},
                                        //{'P', {1,1}, {2,2}, false},
                                        //{'P', {3,4}, {-1,-1}, true},
                                        //{'S', {5,5}, {3,4}, false},
                                        //{'P', {6,4}, {-1,-1}, true},
                                        //{'L', {4,6}, {6,4}, false},
                                        //{'Q', {7,3}, {5,1}, false},
                                        //{'L', {0,5}, {3,2}, false},
                                        //{'P', {2,2}, {-1,-1}, true},
                                        //{'S', {3,4}, {2,2}, false},
                                        //{'L', {6,4}, {4,2}, false},
                                        //{'T', {7,5}, {7,4}, false},
                                        //{'K', {0,4}, {0,5}, false},
                                        //{'L', {3,6}, {5,4}, false},
                                        //{'Q', {5,1}, {-1,-1}, true},
                                        //{'L', {4,2}, {5,1}, false},
                                        //{'L', {3,2}, {-1,-1}, true},
                                        //{'L', {5,4}, {3,2}, false},
                                        //{'K', {0,5}, {0,6}, false},
                                        //{'S', {2,2}, {1,4}, false},
                                        //{'K', {0,6}, {0,5}, false},
                                        //{'P', {3,3}, {-1,-1}, true},
                                        //{'S', {1,4}, {3,3}, false},
                                       };

    //std::cout << "Next move: \nPiece Type:" << next_move.piece_type << "\nFrom: (" << next_move.start_position[0] << ","
    //          << next_move.start_position[1] << ")\nTo: (" << next_move.target_position[0] << "," << next_move.target_position[1] << ")" << std::endl;

    return move_list;
}




