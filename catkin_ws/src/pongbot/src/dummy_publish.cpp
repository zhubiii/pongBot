#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <pongbot/WristGoal.h>


/**
 * publish dummy joint angles to test the driver
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_publish");

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

    wristmsg.wrist_goal_theta = 0;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    ROS_INFO("Publishing Thetas!");
    arm_pub.publish(armmsg);
    wrist_pub.publish(wristmsg);

    ros::spinOnce();
    r.sleep();
    count++;
  }
  return 0;
}