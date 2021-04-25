#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_sdk.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0
#define NUM_JOINTS  4

// Function prototypes
void checkCommResult(int dxl_comm_result, dynamixel::PacketHandler *packetHandler, uint8_t dxl_error);
int checkJoints(uint32_t joint_pos[], uint32_t goal_pos[], int step[], int time_slice, bool restep);

int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "pongbot_demo");

    ros::NodeHandle n;

    // Get all parameters from the yaml config file
    std::string DEVICENAME;
    int DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, BAUDRATE;
    float PROTOCOL_VERSION;
    int ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ADDR_SPEED;
    int JOINT_PAN_ID, JOINT_TILT_ID, JOINT_ELBOW_ID, JOINT_PADDLE_ID;
    int JOINT_PAN_ZERO_CONFIG, JOINT_TILT_ZERO_CONFIG, JOINT_ELBOW_ZERO_CONFIG, JOINT_PADDLE_ZERO_CONFIG;

    n.getParam("usb_port", DEVICENAME);
    n.getParam("dxl_minimum_position_value", DXL_MINIMUM_POSITION_VALUE);
    n.getParam("dxl_maximum_position_value", DXL_MAXIMUM_POSITION_VALUE);
    n.getParam("dxl_baud_rate", BAUDRATE);
    n.getParam("protocol_version", PROTOCOL_VERSION);
    n.getParam("addr_torque_enable", ADDR_TORQUE_ENABLE);
    n.getParam("addr_goal_position", ADDR_GOAL_POSITION);
    n.getParam("addr_present_position", ADDR_PRESENT_POSITION);
    n.getParam("addr_speed", ADDR_SPEED);
    n.getParam("joint_pan/ID", JOINT_PAN_ID);
    n.getParam("joint_tilt/ID", JOINT_TILT_ID);
    n.getParam("joint_elbow/ID", JOINT_ELBOW_ID);
    n.getParam("joint_paddle/ID", JOINT_PADDLE_ID);
    n.getParam("joint_pan/zero_config", JOINT_PAN_ZERO_CONFIG);
    n.getParam("joint_tilt/zero_config", JOINT_TILT_ZERO_CONFIG);
    n.getParam("joint_elbow/zero_config", JOINT_ELBOW_ZERO_CONFIG);
    n.getParam("joint_paddle/zero_config", JOINT_PADDLE_ZERO_CONFIG);

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str());

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint8_t dxl_error = 0;                          // Dynamixel error

    // Open port
    if (portHandler->openPort())
    {
        ROS_INFO("Succeeded to open the port!\n");
    }
    else
    {
        ROS_ERROR("Failed to open the port!\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        ROS_INFO("Succeeded to change the baudrate!\n");
    }
    else
    {
        ROS_ERROR("Failed to change the baudrate!\n");
        return 0;
    }

    // initialize and connect to each motor
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    ROS_INFO("All motors connected successfully \n");

    // Change operating speed of motors
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_SPEED, 200, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_SPEED, 200, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_SPEED, 200, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_SPEED, 200, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
    ROS_INFO("Initialized motor speed \n");

    // present positions
    uint32_t joint_pos[4];
    // TODO: get goal from subscribing to a topic
    uint32_t goal_pos[4] = {JOINT_PAN_ZERO_CONFIG, JOINT_TILT_ZERO_CONFIG, JOINT_ELBOW_ZERO_CONFIG, JOINT_PADDLE_ZERO_CONFIG};
    // Array holding the step sizes for each joint in the next cycle
    int step[4];
    // step goal
    uint32_t sgoal[4];
    uint32_t tmp;

    // x ms to get to goal / y ms time frame
    int time_frame = 6;
    int time_to_goal = 10;
    int time_slice = time_to_goal/time_frame;
    double rate = time_frame * time_to_goal;

    bool restep;

    int count = 0;
    ros::Rate r(rate);
    while(n.ok()){
        // Read control pan motor present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[0], &dxl_error);
        checkCommResult(dxl_comm_result, packetHandler, dxl_error);

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[1], &dxl_error);
        checkCommResult(dxl_comm_result, packetHandler, dxl_error);

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[2], &dxl_error);
        checkCommResult(dxl_comm_result, packetHandler, dxl_error);

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[3], &dxl_error);
        checkCommResult(dxl_comm_result, packetHandler, dxl_error);

        // recalculate every 
        restep = !(count%time_slice);
        if (!checkJoints(joint_pos, goal_pos, step, time_slice, restep))
        {
            ROS_INFO("Moving Joints \n");

            for (size_t i=0; i<NUM_JOINTS; i++)
            {
                if ((tmp=joint_pos[i]+step[i]) > goal_pos[i]) // prevent overstepping
                {
                    ROS_WARN("SETTING MOTOR %d GOAL POS", i);
                    sgoal[i] = goal_pos[i];
                }else{
                    sgoal[i] = tmp;
                }
            }

            // Writing the goal position of the arm
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_GOAL_POSITION, sgoal[0], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_GOAL_POSITION, sgoal[1], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_GOAL_POSITION, sgoal[2], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_GOAL_POSITION, sgoal[3], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);
        }
        count++;
        r.sleep();
    }
    // Close port
    portHandler->closePort();
    return 0;
}

/**
 * @brief Function to check the dynamixel comm result
 * @param dxl_comm_result, *packetHandler, dxl_error
 * */
void checkCommResult(int dxl_comm_result, dynamixel::PacketHandler *packetHandler, uint8_t dxl_error)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        ROS_WARN(packetHandler->getRxPacketError(dxl_error));
    }
}

/**
 * Function to check joint angles and see if they match the desired goal position, will set step array
 * for motors next goal_pos per time frame
 * 
 * PARAM: Takes a joint position array, goal position array, and array specifying the next step for joints
 * 
 * RETURN: 0 if joint angles do not match, 1 if they do
 * 
 * TODO: optimize memory access
 * */
int checkJoints(uint32_t joint_pos[], uint32_t goal_pos[], int step[], int time_slice, bool restep)
{
    int moe = 5; // margin of error 
    int oob = 1023; // out of bounds, check if motors give false joint position reading
    bool match = true;
    for (size_t i=0; i<NUM_JOINTS; i++)
    {
        int error = goal_pos[i] - joint_pos[i];
        ROS_INFO("goal: %d", goal_pos[i]);
        ROS_INFO("error: %d", error);
        // if the error between goal and joint positions is greater than allowed margin of error
        if (abs(error) > oob) {
            ROS_WARN("MOTOR READ ERROR");
            return 1;
        }else if (abs(error) > moe)
        {
            match = false;
            // Make the step size on the order of time slices
            // update is 60hz so we need to calculate how much time (in ms) we want our motor to 
            // make it to the goal position. Then divide that by 6. Call this time_slice
            // Our step then is: (goal_pos - joint_pos)/time_slice
            if (restep)
            {
                int step_size = error/time_slice;
                if (step_size < 1 && step_size > -1 )
                {
                    ROS_WARN("Desired Step Size too small");
                }else
                {
                    step[i] = step_size;
                }
            }
        }
    }
    return match;
}