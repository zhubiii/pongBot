#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <pongbot/HomoTransformMatrix.h>
#include <stdlib.h>


void updateTransform(const pongbot::HomoTransformMatrix::ConstPtr& msg);

std::vector<std::vector<double>> X (4, std::vector<double>(4));

/**
 * Inverse Kinematics
 * subscribes to get desired end effector homogeneous transform matrix X
 * publishes the joint thetas to get there
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pongbot_ikin");

  ros::NodeHandle n;

  ros::Subscriber arm_sub = n.subscribe("end_effector_transform", 1000, updateTransform);
  ros::Publisher arm_pub = n.advertise<pongbot::JointGoal>("arm_goal_theta", 1000);

  double px, py, pz, D, s3, c3;
  // TODO: expose param also maybe in driver
  const double a2 = 149.97;
  const double a3 = 236.92;
  double theta1, theta2, theta3;
  ros::Rate r(100.0);
  int count = 0;
  while (ros::ok())
  {
    pongbot::JointGoal armmsg;

    px = X[0][3];
    py = X[1][3];
    pz = X[2][3];

    D = (pow(px,2) + pow(py,2) + pow(pz,2) - pow(a2,2) - pow(a3,2)) / (2*a2*a3);

    theta1 = atan2(py,px);

    // take pos or neg?
    theta3 = atan2(abs(sqrt(1-pow(D,2))), D);

    s3 = sin(theta3);
    c3 = cos(theta3);
    theta2 = atan2(pz, sqrt(pow(px,2) + pow(py,2))) - atan2(a3*s3, a2+a3*c3);


    armmsg.joint_thetas.push_back(theta1);
    armmsg.joint_thetas.push_back(theta2);
    armmsg.joint_thetas.push_back(theta3);

    ROS_INFO("Publishing Thetas!");
    arm_pub.publish(armmsg);

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