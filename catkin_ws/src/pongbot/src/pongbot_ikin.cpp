#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <pongbot/WristGoal.h>
#include <pongbot/HomoTransformMatrix.h>
#include <stdlib.h>


void updateTransform(const pongbot::HomoTransformMatrix::ConstPtr& msg);

std::vector<std::vector<double>> X (4, std::vector<double>(4))

/**
 * Inverse Kinematics
 * subscribes to get desired end effector homogeneous transform matrix X
 * publishes the joint thetas to get there
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pongbot_ikin");

  ros::NodeHandle n;

  ros::Publisher arm_pub = n.advertise<pongbot::JointGoal>("arm_goal_theta", 1000);
  ros::Publisher wrist_pub = n.advertise<pongbot::WristGoal>("wrist_goal_theta", 1000);

  ros::Rate r(100.0);
  int count = 0;
  while (ros::ok())
  {
    pongbot::JointGoal armmsg;
    pongbot::WristGoal wristmsg;
    armmsg.joint_thetas.push_back(M_PI/6);
    armmsg.joint_thetas.push_back(M_PI/6);
    armmsg.joint_thetas.push_back(M_PI/6);

    wristmsg.wrist_goal_theta = M_PI/4;

    ROS_INFO("Publishing Thetas!");
    arm_pub.publish(armmsg);
    wrist_pub.publish(wristmsg);

    ros::spinOnce();
    r.sleep();
    count++;
  }
  return 0;
}

void updateTransform(const pongbot::HomoTransformMatrix::ConstPtr& msg)
{
    size_t i,j,count = 0;
    for (i=0; i<4; i++)
        for (j=0; j<4; j++)
            X[i][j] = msg->X[count++];
}