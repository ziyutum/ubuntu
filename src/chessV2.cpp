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
#define BOARD_FIELD_SPACING 0.045      // Distance between the center of chess board fields
#define INITIAL_DX_TO_BOARD_DATUM 0.08 // Distance between initial x position of EE and boards (0,0)
#define INITIAL_DY_TO_BOARD_DATUM 0.17
#define HEIGHT_PRAWN 0.035
#define BASE_PLATE_HEIGHT 0.02
#define CHESS_BOARD_HEIGHT 0.008
#define HEIGHT_QUEEN_KING  0.06
#define HEIGHT_BISHOP 0.047
#define HEIGHT_KNIGHT 0.053
#define HEIGHT_TOWER 0.04

// Include predefined controllers and operational variables
#include "skill_lib.h"

// Chess game specific data types
struct chess_move
{
    char piece_type; // The piece, necessary to calculate gripper height and width
    std::array<int,2> start_position;  // Starting and target position on the board
    std::array<int,2> target_position; // defined as X,Y coordinates.
    bool isRemove; // Indicates if in the next move the piece that is removed will be hit
};

// Forward declarations
std::array<double, 2> calc_dX_dX(int X1, int Y1, int X2 = 0, int Y2 = 0, bool fromInitial=false, int fromRemoval=-1);
double calc_dZ(char piece_type, double current_z);
void MoveChessPieceOnBoard(franka::Robot& robot,franka::Gripper& gripper, chess_move& move, chess_move& last_move,bool fromInit = false);
void RemoveChessPieceFromBoard(franka::Robot& robot,franka::Gripper& gripper, chess_move& move, chess_move& last_move);
void replayChessMatch(std::vector<chess_move>& replayMoves, franka::Robot& robot, franka::Gripper& gripper);
bool move_to_init(franka::Robot& rob);
std::vector<chess_move> generate_move_list();

// ENTRY POINT
int main(int argc, char** argv)
{
    std::cout << "Stared chess programm V2" << std::endl;

    if(argc != 2)
    {
        std::cerr << "FCI I.P. not specified only argv[0]: " << argv[0] << std::endl;
    }

    try
    {
        // Create Robot instance
        franka::Robot robot(argv[1]);
        // Create the Gripper instance
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
        std::stringstream buffer;
        buffer << robot_state_init;
        std::string t = buffer.str();
        std::cout<< buffer.str().c_str() << std::endl;
        std::cout << sizeof(buffer.str().c_str()) << std::endl;
        std::cout << buffer.str().length() << std::endl;
        //std::array<double, 16> end_effector_pos_init = robot_state_init.O_T_EE;
        //std::cout << "Robot End-Effector position: x=" << end_effector_pos_init[12]
        //          << " y=" << end_effector_pos_init[13] << " z=" << end_effector_pos_init[14] << std::endl;

        //---------------------------//
        //  Set-up TCP connection    //
        //---------------------------//
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
        client_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        std::cout << "Server program, client_socket: "  << client_socket << std::endl;

        // Send a welcome message
        char welcome_message[100] = "Welcome from FCI control program!";
        std::cout << "Before" << std::endl;
        send(client_socket, welcome_message, sizeof(welcome_message), MSG_CONFIRM);
        std::cout << "Sent out the welcome message" << std::endl;

        //---------- End of TCP Set-up ----------------------

        // Generate a list of chess moves
        std::vector<chess_move> all_moves = generate_move_list();

       // Top-Level entry point into the robots capabilities
        replayChessMatch(all_moves, robot, gripper);

    }
    catch (franka::Exception franka_ex)
    {
        std::cout << franka_ex.what() << std::endl;
    }
    return 0;
}

void replayChessMatch(std::vector<chess_move>& replayMoves, franka::Robot& robot, franka::Gripper& gripper){
    for(int i = 0; i < replayMoves.size(); i++){
        std::cout << "Next move: " << replayMoves[i].piece_type << "  (" << replayMoves[i].start_position[0] << "," << replayMoves[i].start_position[1] << ") " << " (" << replayMoves[i].target_position[0] << "," << replayMoves[i].target_position[1] << ")" <<std::endl;
        if(i==0){
            // First call to move from initial position to first pock up position
            MoveChessPieceOnBoard(robot, gripper, replayMoves[i], replayMoves[i], true);
        }
        else {
            if(replayMoves[i].isRemove==false)
                MoveChessPieceOnBoard(robot, gripper, replayMoves[i], replayMoves[i-1]);
            else
               RemoveChessPieceFromBoard(robot, gripper, replayMoves[i], replayMoves[i-1]);
        }
    }
}

// @capability_description: Moves a chess piece on the board by making use of the basic skills
//                          to perform a cartesian motion and opening and closing the gripper.
void MoveChessPieceOnBoard(franka::Robot& robot,franka::Gripper& gripper, chess_move& move, chess_move& last_move, bool fromInit){
    // 1. Open the gripper
    GripperOpeningWidth = 0.03; GripperOpeningSpeed = 0.1;
    gripper.move(GripperOpeningWidth, GripperOpeningWidth);
    std::cout << "Opened Gripper 1st" << std::endl;
    // 2. Move to pick up position
    if(fromInit){
        // Move from the known initial position to the first starting position
        std::array<double, 2> next_increment = calc_dX_dX(move.start_position[0], move.start_position[1], -1, -1, true);
        std::cout << "Init increments :" << next_increment[0] << " "  << next_increment[1] << std::endl;
        x_increment = next_increment[0];
        y_increment = next_increment[1];
        robot.control(CartesianMotionFunction);
    }else {
        // Assume we one move after the other, i.e. we know can compute the next increment from the
        // end position of the last move. Extension: function to calc board pos. or increment based
        // on current robot EE position information.
        std::array<double, 2> next_increment = calc_dX_dX(last_move.target_position[0], last_move.target_position[1], move.start_position[0], move.start_position[1]);
        std::cout << "To pick-up increments :" << next_increment[0] << " "  << next_increment[1] << std::endl;
        x_increment = next_increment[0];
        y_increment = next_increment[1];
        robot.control(CartesianMotionFunction);
    }
    // 3. Move down
    double next_dz = calc_dZ(move.piece_type, robot.readOnce().O_T_EE[14]);
    std::cout << "Move down 1st: " << next_dz << std::endl;
    x_increment = 0;
    y_increment = 0;
    z_increment = next_dz;
    robot.control(CartesianMotionFunction);
    // 4. Close Gripper
    if(move.piece_type == 'T' || move.piece_type == 'K' || move.piece_type == 'Q' || move.piece_type == 'L')
    {
        GripperClosingWidth = 0.02;
        GripperClosingSpeed = 0.04;
        gripper.grasp(GripperClosingWidth, GripperClosingSpeed, GripperClosingForce);
    }
    else
    {
        GripperClosingWidth = 0.01;
        GripperClosingSpeed = 0.04;
        gripper.grasp(GripperClosingWidth, GripperClosingSpeed, GripperClosingForce);
    }
    // 5. Move up again
    x_increment = 0;
    y_increment = 0;
    z_increment = -0.2; // CONSTANT MOVING HEIGTH
    robot.control(CartesianMotionFunction);
    // 6. Move to the new position
    std::array<double, 2> next_increment = calc_dX_dX(move.start_position[0], move.start_position[1], move.target_position[0], move.target_position[1]);
    std::cout << "To target pos increments :" << next_increment[0] << " "  << next_increment[1] << std::endl;
    x_increment = next_increment[0];
    y_increment = next_increment[1];
    z_increment = 0.0;
    robot.control(CartesianMotionFunction);
    // 7. Move down
    double next_next_dz = calc_dZ(move.piece_type, robot.readOnce().O_T_EE[14]);
    std::cout << "Move down 2nd: " << next_next_dz << std::endl;
    x_increment = 0;
    y_increment = 0;
    z_increment = next_next_dz;
    robot.control(CartesianMotionFunction);
    // 8. Open gripper
    GripperOpeningWidth = 0.03; GripperOpeningSpeed = 0.1;
    gripper.move(GripperOpeningWidth, GripperOpeningWidth);
    // 9. Move up again
    x_increment = 0;
    y_increment = 0;
    z_increment = -0.2; // CONSTANT MOVING HEIGTH
    robot.control(CartesianMotionFunction);
}

// @capability_description: Removes chess piece from board to a predefined location
//                          and then moves back to the position from which the piece
//                          was removed (implementation specific behavior)
void RemoveChessPieceFromBoard(franka::Robot& robot,franka::Gripper& gripper, chess_move& move, chess_move& last_move){
    // 1. Open the gripper
    GripperOpeningWidth = 0.03; GripperOpeningSpeed = 0.1;
    gripper.move(GripperOpeningWidth, GripperOpeningWidth);
    std::cout << "Opened Gripper 1st" << std::endl;
    // 2. Move to pick up position
    // Assume we one move after the other, i.e. we know can compute the next increment from the
    // end position of the last move. Extension: function to calc board pos. or increment based
    // on current robot EE position information.
    std::array<double, 2> next_increment = calc_dX_dX(last_move.target_position[0], last_move.target_position[1], move.start_position[0], move.start_position[1]);
    std::cout << "To pick-up increments :" << next_increment[0] << " "  << next_increment[1] << std::endl;
    x_increment = next_increment[0];
    y_increment = next_increment[1];
    robot.control(CartesianMotionFunction);

    // 3. Move down
    double next_dz = calc_dZ(move.piece_type, robot.readOnce().O_T_EE[14]);
    std::cout << "Move down 1st: " << next_dz << std::endl;
    x_increment = 0;
    y_increment = 0;
    z_increment = next_dz;
    robot.control(CartesianMotionFunction);
    // 4. Close Gripper
    if(move.piece_type == 'T' || move.piece_type == 'K' || move.piece_type == 'Q' || move.piece_type == 'L')
    {
        GripperClosingWidth = 0.02;
        GripperClosingSpeed = 0.04;
        gripper.grasp(GripperClosingWidth, GripperClosingSpeed, GripperClosingForce);
    }
    else
    {
        GripperClosingWidth = 0.01;
        GripperClosingSpeed = 0.04;
        gripper.grasp(GripperClosingWidth, GripperClosingSpeed, GripperClosingForce);
    }
    // 5. Move up again
    x_increment = 0;
    y_increment = 0;
    z_increment = -0.2; // CONSTANT MOVING HEIGTH
    robot.control(CartesianMotionFunction);
    // 6. Move to the remove position
    std::array<double, 2> next_next_increment = calc_dX_dX(move.start_position[0], move.start_position[1], -1, -1, false, 1);
    std::cout << "To target pos increments :" << next_next_increment[0] << " "  << next_next_increment[1] << std::endl;
    x_increment = next_next_increment[0];
    y_increment = next_next_increment[1];
    z_increment = 0.0;
    robot.control(CartesianMotionFunction);
    // 7. Open gripper
    GripperOpeningWidth = 0.03; GripperOpeningSpeed = 0.1;
    gripper.move(GripperOpeningWidth, GripperOpeningWidth);
    // 8. Move back to last valid board position (from where piece was taken)
    std::cout << "Moving back to last known position" << std::endl;
    x_increment = -next_next_increment[0];
    y_increment = -next_next_increment[1];
    z_increment = 0.0;
    robot.control(CartesianMotionFunction);
}


// @brief Given to positions on the chessboard calculates the X and Y
//        distances between these fields. There are some special cases when
//        the robot comes from the init position or from the removal place.
std::array<double, 2> calc_dX_dX(int X1, int Y1, int X2, int Y2,  bool fromInitial, int fromRemoval){

    std::array<double, 2> dXdY;

    if(fromInitial){
        dXdY[0] = INITIAL_DX_TO_BOARD_DATUM+BOARD_FIELD_SPACING*X1;
        dXdY[1] = -INITIAL_DY_TO_BOARD_DATUM+BOARD_FIELD_SPACING*Y1;
    }
    else if(fromRemoval != -1){
        // fromRemoval case:
        // For the 32 chess pieces there are 30 fixed removal places with known
        // coordinates, so that the robot can put the figures back on the board
        dXdY[0] = -0.20-BOARD_FIELD_SPACING*X1;
        dXdY[1] = -0.05-BOARD_FIELD_SPACING*Y1;
    }
    else{
        // Default from one chess field to another
        dXdY[0] = BOARD_FIELD_SPACING*(X2-X1);
        dXdY[1] = BOARD_FIELD_SPACING*(Y2-Y1);
    }
    return dXdY;
}

// @brief Calculates the z position the gripper must have to grab a certain type of chess
//        piece. Distance is the same for picking up and placing.
double calc_dZ(char piece_type, double current_z){
    double dZ = 0;
    if(piece_type == 'P')
        dZ = current_z + BASE_PLATE_HEIGHT - HEIGHT_PRAWN - CHESS_BOARD_HEIGHT;
    else if(piece_type == 'Q' || piece_type=='K')
        dZ = current_z + BASE_PLATE_HEIGHT - HEIGHT_QUEEN_KING - CHESS_BOARD_HEIGHT;
    else if(piece_type=='T')
        dZ = current_z + BASE_PLATE_HEIGHT - HEIGHT_TOWER - CHESS_BOARD_HEIGHT;
    else if(piece_type=='L')
        dZ = current_z + BASE_PLATE_HEIGHT - HEIGHT_BISHOP- CHESS_BOARD_HEIGHT;
    else if(piece_type == 'S')
        dZ = current_z + BASE_PLATE_HEIGHT - HEIGHT_KNIGHT- CHESS_BOARD_HEIGHT;
    return dZ;
}

// @brief Helper function that handels the initalization procedure
bool move_to_init(franka::Robot& rob){
    // Set a speedfactor
    double speedfactor = 0.3;
    // Set initial position by specifying joint angles
    std::array<double, 7> safepos = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // Get an already implemented motion generator
    MotionGenerator motiongenerator(speedfactor, safepos);
    // Call the robot control with the motion generator to execute
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

std::vector<chess_move> generate_move_list()
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

    // For remove moves the coordinate from where the piece is removed is given twice
    std::vector<chess_move> move_list = { {'S', {0,6}, {2,5}, false},
                                        {'S', {7,6}, {5,5}, false},
                                        {'P', {1,2}, {3,2}, false},
                                        {'P', {6,6}, {5,6}, false},
                                        {'S', {0,1}, {2,2}, false},
                                        {'L', {7,5}, {6,6}, false},
                                        {'P', {1,3}, {3,3}, false},
//                                        {'T', {7,7}, {4,8}, false},
//                                        {'K', {7,4}, {7,6}, false},
//                                        {'T', {4,8}, {7,5}, false},
                                        {'T', {7,7}, {7,5}, false},
                                        {'K', {7,4}, {7,6}, false},
                                        {'L', {0,2}, {3,5}, false},
                                        {'P', {6,3}, {4,3}, false},
                                        {'Q', {0,3}, {2,1}, false},
                                        {'P', {3,2}, {3,2}, true}, // {'P', {3,2}, {-1,-1}, true},
                                        {'P', {4,3}, {3,2}, false},
                                        {'P', {3,2}, {3,2}, true}, // {'P', {3,2}, {-1,-1}, true},
                                        {'Q', {2,1}, {3,2}, false},
                                        {'P', {6,2}, {5,2}, false},
                                        {'P', {1,4}, {3,4}, false},
                                        {'S', {7,1}, {6,3}, false},
                                        {'T', {0,0}, {0,3}, false},
                                        {'S', {6,3}, {5,1}, false},
                                        {'Q', {3,2}, {4,2}, false},
                                        {'L', {7,2}, {3,6}, false},
                                        {'L', {3,5}, {4,6}, false},
                                        {'S', {5,1}, {3,0}, false},
                                        {'Q', {4,2}, {2,0}, false},
                                        {'S', {2,2}, {2,2}, true},
                                        {'S', {3,0}, {2,2}, false},
                                        {'S', {2,2}, {2,2}, true},
                                        {'P', {1,1}, {2,2}, false},
                                        {'P', {3,4}, {3,4}, true},
                                        {'S', {5,5}, {3,4}, false},
                                        {'P', {6,4}, {6,4}, true},
                                        {'L', {4,6}, {6,4}, false},
                                        {'Q', {7,3}, {5,1}, false},
                                        {'L', {0,5}, {3,2}, false},
                                        {'P', {2,2}, {2,2}, true},
                                        {'S', {3,4}, {2,2}, false},
                                        //{'L', {6,4}, {4,2}, false},
                                        //{'T', {7,5}, {7,4}, false},
                                        //{'K', {0,4}, {0,5}, false},
                                        //{'L', {3,6}, {5,4}, false},
                                        //{'Q', {5,1}, {5,1}, true},
                                        //{'L', {4,2}, {5,1}, false},
                                        //{'L', {3,2}, {3,2}, true},
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

