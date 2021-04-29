#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <pongbot/WristGoal.h>

uint32_t joint_pos[4] = {512, 819, 819, 512};

/**
 * publish dummy joint angles to test the driver
 */

void updatePos(const pongbot::JointGoal::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_publish");

  ros::NodeHandle n;

  ros::Publisher arm_pub = n.advertise<pongbot::JointGoal>("arm_goal", 1000);
  ros::Publisher wrist_pub = n.advertise<pongbot::WristGoal>("wrist_goal", 1000);
  ros::Subscriber sub = n.subscribe("joint_pos", 1000, updatePos);

  ros::Rate r(100.0);

  bool plus = true;
  int count = 0;
  int angle[4] = {joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3]};
  while (ros::ok())
  {
    pongbot::JointGoal armmsg;
    pongbot::WristGoal wristmsg;

    if ((count%250==0 && count!=0)){
        plus = !plus;
    }

    if (count > 0) {
        armmsg.joints.push_back(angle[0]+=1);
        armmsg.joints.push_back(angle[1]-=1);
        armmsg.joints.push_back(angle[2]-=1);
        wristmsg.wrist_goal = angle[3]+=1;
    }else{
        armmsg.joints.push_back(angle[0]-=1);
        armmsg.joints.push_back(angle[1]+=1);
        armmsg.joints.push_back(angle[2]+=1);
        wristmsg.wrist_goal = angle[3]-=1;
    }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    ROS_INFO("Publishing Goal!");
    arm_pub.publish(armmsg);
    wrist_pub.publish(wristmsg);

    ros::spinOnce();

    r.sleep();
    if (plus){
        count++;
    }else{
        count--;
    }
  }
  return 0;
}

/** Callback function for getting the goal position from subscribed topic
 * */
void updatePos(const pongbot::JointGoal::ConstPtr& msg)
{
    for (size_t i=0; i<4; i++)
        joint_pos[i] = msg->joints.at(i);
}