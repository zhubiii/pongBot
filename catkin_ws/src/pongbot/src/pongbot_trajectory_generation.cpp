#include "ros/ros.h"
#include <pongbot/HomoTransformMatrix.h>


/**
 * publish trajectory generation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generation");

  ros::NodeHandle n;
  ros::Publisher HTMpub = n.advertise<pongbot::HomoTransformMatrix>("end_effector_transform", 1000);

  ros::Rate r(100.0);
  int count = 0;
  while (ros::ok())
  {
    pongbot::HomoTransformMatrix HTMmsg;

    HTMmsg.X = {0,0,1,379.93, 1,0,0,0, 0,1,0,84.455, 0,0,0,1};

    HTMpub.publish(HTMmsg);

    ros::spinOnce();
    r.sleep();
    count++;
  }
  return 0;
}