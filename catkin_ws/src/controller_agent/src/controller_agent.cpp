#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_sdk.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0

int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "controller_agent");

    ros::NodeHandle n;

    // Get all parameters from the yaml config file
    std::string DEVICENAME;
    int DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE, BAUDRATE;
    float PROTOCOL_VERSION;
    int ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION;
    int AGENT_PAN_ID, AGENT_TILT_ID, CONTROL_PAN_ID, CONTROL_TILT_ID;

    n.getParam("usb_port", DEVICENAME);
    n.getParam("dxl_minimum_position_value", DXL_MINIMUM_POSITION_VALUE);
    n.getParam("dxl_maximum_position_value", DXL_MAXIMUM_POSITION_VALUE);
    n.getParam("dxl_baud_rate", BAUDRATE);
    n.getParam("protocol_version", PROTOCOL_VERSION);
    n.getParam("addr_torque_enable", ADDR_TORQUE_ENABLE);
    n.getParam("addr_goal_position", ADDR_GOAL_POSITION);
    n.getParam("addr_present_position", ADDR_PRESENT_POSITION);
    n.getParam("agent_pan/ID", AGENT_PAN_ID);
    n.getParam("agent_tilt/ID", AGENT_TILT_ID);
    n.getParam("control_pan/ID", CONTROL_PAN_ID);
    n.getParam("control_tilt/ID", CONTROL_TILT_ID);

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

    // present positions
    int32_t agent_pan_present_position = 0, agent_tilt_present_position = 0;
    int32_t control_pan_present_position = 0, control_tilt_present_position = 0;

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
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, AGENT_PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        ROS_WARN(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        ROS_INFO("Agent Pan motor has been successfully connected \n");
    } 

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, AGENT_TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        ROS_WARN(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        ROS_INFO("Agent Tilt motor has been successfully connected \n");
    } 

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, CONTROL_PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        ROS_WARN(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        ROS_INFO("Controller Pan motor has been successfully connected \n");
    } 

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, CONTROL_TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        ROS_WARN(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        ROS_INFO("Controller Tilt motor has been successfully connected \n");
    } 

    int count = 0;
    ros::Rate r(1.0);
    while(n.ok()){
        do
        {
            // Read control pan motor present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, CONTROL_PAN_ID, ADDR_PRESENT_POSITION, (uint32_t*)&control_pan_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

            // Read Control tilt motor present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, CONTROL_TILT_ID, ADDR_PRESENT_POSITION, (uint32_t*)&control_tilt_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

            // Read Agent Pan motor present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, AGENT_PAN_ID, ADDR_PRESENT_POSITION, (uint32_t*)&agent_pan_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

            // Read Agent tilt motor present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, AGENT_TILT_ID, ADDR_PRESENT_POSITION, (uint32_t*)&agent_tilt_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

            // Write agent pan goal position
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, AGENT_PAN_ID, ADDR_GOAL_POSITION, control_pan_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

            // Write agent tilt goal position
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, AGENT_TILT_ID, ADDR_GOAL_POSITION, control_tilt_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN(packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                ROS_WARN(packetHandler->getRxPacketError(dxl_error));
            }

        }while(abs(control_pan_present_position-agent_pan_present_position) != 0 && abs(control_tilt_present_position-agent_tilt_present_position) != 0);
        count++;
        r.sleep();
    }
    // Close port
    portHandler->closePort();
    return 0;
}



