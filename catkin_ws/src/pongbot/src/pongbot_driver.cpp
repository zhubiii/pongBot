#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_sdk.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <pongbot/JointGoal.h>
#include <pongbot/WristGoal.h>

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0
#define NUM_JOINTS  4

uint32_t joint_pos[4];
// init values with zero config
uint32_t goal_pos[3] = {512, 750, 819};
uint32_t wrist_goal = 512;

// Get all parameters from the yaml config file
std::string DEVICENAME;
int DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, BAUDRATE;
float PROTOCOL_VERSION;
int ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION, ADDR_SPEED, ADDR_CW_COMPLIANCE_SLOPE, ADDR_CCW_COMPLIANCE_SLOPE;
int JOINT_PAN_ID, JOINT_TILT_ID, JOINT_ELBOW_ID, JOINT_PADDLE_ID;
int JOINT_PAN_SPEED, JOINT_TILT_SPEED, JOINT_ELBOW_SPEED, JOINT_PADDLE_SPEED;
int JOINT_PAN_CCW_COMPLIANCE_SLOPE, JOINT_TILT_CCW_COMPLIANCE_SLOPE, JOINT_ELBOW_CCW_COMPLIANCE_SLOPE, JOINT_PADDLE_CCW_COMPLIANCE_SLOPE;
int JOINT_PAN_CW_COMPLIANCE_SLOPE, JOINT_TILT_CW_COMPLIANCE_SLOPE, JOINT_ELBOW_CW_COMPLIANCE_SLOPE, JOINT_PADDLE_CW_COMPLIANCE_SLOPE;

// Function prototypes
void checkCommResult(int dxl_comm_result, dynamixel::PacketHandler *packetHandler, uint8_t dxl_error);
void updateGoalArm(const pongbot::JointGoal::ConstPtr& msg);
void updateGoalWrist(const pongbot::WristGoal::ConstPtr& msg);
void getJointPos(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_comm_result, uint8_t dxl_error);
void setJointPos(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_comm_result, uint8_t dxl_error);

int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "pongbot_driver");

    ros::NodeHandle n;


    n.getParam("usb_port", DEVICENAME);
    n.getParam("dxl_minimum_position_value", DXL_MINIMUM_POSITION_VALUE);
    n.getParam("dxl_maximum_position_value", DXL_MAXIMUM_POSITION_VALUE);
    n.getParam("dxl_baud_rate", BAUDRATE);
    n.getParam("protocol_version", PROTOCOL_VERSION);
    n.getParam("addr_torque_enable", ADDR_TORQUE_ENABLE);
    n.getParam("addr_goal_position", ADDR_GOAL_POSITION);
    n.getParam("addr_present_position", ADDR_PRESENT_POSITION);
    n.getParam("addr_speed", ADDR_SPEED);
    n.getParam("addr_cw_compliance_slope", ADDR_CW_COMPLIANCE_SLOPE);
    n.getParam("addr_ccw_compliance_slope", ADDR_CCW_COMPLIANCE_SLOPE);
    n.getParam("joint_pan/ID", JOINT_PAN_ID);
    n.getParam("joint_tilt/ID", JOINT_TILT_ID);
    n.getParam("joint_elbow/ID", JOINT_ELBOW_ID);
    n.getParam("joint_paddle/ID", JOINT_PADDLE_ID);
    n.getParam("joint_pan/speed", JOINT_PAN_SPEED);
    n.getParam("joint_tilt/speed", JOINT_TILT_SPEED);
    n.getParam("joint_elbow/speed", JOINT_ELBOW_SPEED);
    n.getParam("joint_paddle/speed", JOINT_PADDLE_SPEED);
    n.getParam("joint_pan/ccw_compliance_slope", JOINT_PAN_CCW_COMPLIANCE_SLOPE);
    n.getParam("joint_tilt/ccw_compliance_slope", JOINT_TILT_CCW_COMPLIANCE_SLOPE);
    n.getParam("joint_elbow/ccw_compliance_slope", JOINT_ELBOW_CCW_COMPLIANCE_SLOPE);
    n.getParam("joint_paddle/ccw_compliance_slope", JOINT_PADDLE_CCW_COMPLIANCE_SLOPE);
    n.getParam("joint_pan/cw_compliance_slope", JOINT_PAN_CW_COMPLIANCE_SLOPE);
    n.getParam("joint_tilt/cw_compliance_slope", JOINT_TILT_CW_COMPLIANCE_SLOPE);
    n.getParam("joint_elbow/cw_compliance_slope", JOINT_ELBOW_CW_COMPLIANCE_SLOPE);
    n.getParam("joint_paddle/cw_compliance_slope", JOINT_PADDLE_CW_COMPLIANCE_SLOPE);

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
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_SPEED, JOINT_PAN_SPEED, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_SPEED, JOINT_TILT_SPEED, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_SPEED, JOINT_ELBOW_SPEED, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_SPEED, JOINT_PADDLE_SPEED, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
    ROS_INFO("Initialized motor speed \n");

    // Change PID controls to allow incremental moves
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_CCW_COMPLIANCE_SLOPE, JOINT_PAN_CCW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_CCW_COMPLIANCE_SLOPE, JOINT_TILT_CCW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_CCW_COMPLIANCE_SLOPE, JOINT_ELBOW_CCW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_CCW_COMPLIANCE_SLOPE, JOINT_PADDLE_CCW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
    ROS_INFO("Initialized CCW Compliance Slope \n");

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_CW_COMPLIANCE_SLOPE, JOINT_PAN_CW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_CW_COMPLIANCE_SLOPE, JOINT_TILT_CW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_CW_COMPLIANCE_SLOPE, JOINT_ELBOW_CW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_CW_COMPLIANCE_SLOPE, JOINT_PADDLE_CW_COMPLIANCE_SLOPE, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
    ROS_INFO("Initialized CW Compliance Slope \n");

    //subcribe to joint_goal topic to get goal_pos
    ros::Subscriber arm_sub = n.subscribe("arm_goal", 1000, updateGoalArm);
    ros::Subscriber wristsub = n.subscribe("wrist_goal", 1000, updateGoalWrist);
    // publish the present position of motors
    ros::Publisher chatter_pub = n.advertise<pongbot::JointGoal>("joint_pos", 1000);

    int count = 0;
    ros::Rate r(100.0);
    while(n.ok()){
        // Read and store joint positions
        getJointPos(packetHandler, portHandler, dxl_comm_result, dxl_error);

        pongbot::JointGoal msg;

        msg.joints.push_back(joint_pos[0]);
        msg.joints.push_back(joint_pos[1]);
        msg.joints.push_back(joint_pos[2]);
        msg.joints.push_back(joint_pos[3]);

        chatter_pub.publish(msg);

        // Writing the goal position of the entire arm
        setJointPos(packetHandler, portHandler, dxl_comm_result, dxl_error);

        count++;
        ros::spinOnce();
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

/** Callback function for getting the goal position from subscribed topic
 * */
void updateGoalArm(const pongbot::JointGoal::ConstPtr& msg){
    //read for all except wrist motor
    for (size_t i=0; i<NUM_JOINTS-1; i++)
        goal_pos[i] = msg->joints.at(i);
}

/** Callback function for getting the goal wrist position from subscribed topic
 * */
void updateGoalWrist(const pongbot::WristGoal::ConstPtr& msg){
    wrist_goal = msg->wrist_goal;
}

/** Reads the position of all the joints and puts them into joint_pos[]
 * */
void getJointPos(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_comm_result, uint8_t dxl_error)
{
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[0], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[1], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[2], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_PRESENT_POSITION, (uint32_t*)&joint_pos[3], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
}

/** Write out goal positions for each joint
 * */
void setJointPos(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler, int dxl_comm_result, uint8_t dxl_error)
{
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PAN_ID, ADDR_GOAL_POSITION, goal_pos[0], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_TILT_ID, ADDR_GOAL_POSITION, goal_pos[1], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_ELBOW_ID, ADDR_GOAL_POSITION, goal_pos[2], &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, JOINT_PADDLE_ID, ADDR_GOAL_POSITION, wrist_goal, &dxl_error);
    checkCommResult(dxl_comm_result, packetHandler, dxl_error);
}