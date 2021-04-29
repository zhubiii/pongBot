#include "ros/ros.h"
#include <pongbot/JointGoal.h>

/**
 * publish dummy joint angles to test the driver
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_publish");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<pongbot::JointGoal>("joint_goal", 1000);

  ros::Rate r(10.0);

  int count = 0;
  while (ros::ok())
  {
    pongbot::JointGoal msg;

    msg.joints.push_back(512+count);
    msg.joints.push_back(819-count);
    msg.joints.push_back(819-count);
    msg.joints.push_back(512+count);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    ROS_INFO("Publishing Goal!");
    chatter_pub.publish(msg);

    ros::spinOnce();

    r.sleep();
    count++;
  }
  return 0;
}