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
int checkJoints(uint32_t joint_pos[], uint32_t goal_pos[], int step[]);

int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "pongbot_demo");

    ros::NodeHandle n;

    // Get all parameters from the yaml config file
    std::string DEVICENAME;
    int DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, BAUDRATE;
    float PROTOCOL_VERSION;
    int ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION;
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

    // present positions
    uint32_t joint_pos[4];
    // TODO: get goal from subscribing to a topic
    uint32_t goal_pos[] = {JOINT_PAN_ZERO_CONFIG, JOINT_TILT_ZERO_CONFIG, JOINT_ELBOW_ZERO_CONFIG, JOINT_PADDLE_ZERO_CONFIG};
    // Array holding the step sizes for each joint in the next cycle
    int step[4];

    int count = 0;
    ros::Rate r(80.0);
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

            
        if (!checkJoints(joint_pos, goal_pos, step))
        {
            ROS_INFO("Moving Joints \n");
            ROS_INFO("JOINT POS: %d", joint_pos[1]);
            ROS_INFO("GOAL POS %d", goal_pos[1]);

            // Writing the goal position of the arm
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_GOAL_POSITION, joint_pos[0]+step[0], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_GOAL_POSITION, joint_pos[1]+step[1], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_GOAL_POSITION, joint_pos[2]+step[2], &dxl_error);
            checkCommResult(dxl_comm_result, packetHandler, dxl_error);

            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_GOAL_POSITION, joint_pos[3]+step[3], &dxl_error);
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
 * Function to check the dynamixel comm result
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
 * Function to check joint angles and see if they match the desired goal position
 * 
 * PARAM: Takes a joint position array, goal position array, and array specifying the next step for joints
 * 
 * RETURN: 0 if joint angles do not match, 1 if they do
 * 
 * TODO: optimize memory access
 * */
int checkJoints(uint32_t joint_pos[], uint32_t goal_pos[], int step[])
{
    bool match = true;
    for (size_t i=0; i<NUM_JOINTS; i++)
    {
        if (joint_pos[i] != goal_pos[i])
        {
            match = false;
            if (joint_pos[i] > goal_pos[i])
            {
                step[i] = -5;
            }else if (joint_pos[i] < goal_pos[i]){
                step[i] = 5;
            }else{
                step[i] = 0;
            }
        }
    }
    return match;
}