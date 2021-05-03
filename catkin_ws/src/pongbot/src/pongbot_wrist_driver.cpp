#include "ros/ros.h"
#include <pongbot/WristGoal.h>


/**
 * publish thetas for wrist
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wrist_driver");

  ros::NodeHandle n;

  ros::Publisher wrist_pub = n.advertise<pongbot::WristGoal>("wrist_goal_theta", 1000);

  ros::Rate r(100.0);
  int count = 0;
  while (ros::ok())
  {
    pongbot::WristGoal wristmsg;
    wristmsg.wrist_goal_theta = count;

    wrist_pub.publish(wristmsg);

    ros::spinOnce();
    r.sleep();
    count++;
  }
  return 0;
}