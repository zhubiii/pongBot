#include "ros/ros.h"
#include <pongbot/JointGoal.h>
#include <pongbot/WristGoal.h>

const double DEGREE = .29;
const double RADIAN = M_PI/180;

/**
 * Takes in theta messages and publishes the corresponding dxl position
 */

void updatePosArm(const pongbot::JointGoal::ConstPtr& msg);
void updatePosWrist(const pongbot::WristGoal::ConstPtr& msg);
void convertTheta(pongbot::JointGoal* jointmsg, pongbot::WristGoal* wristmsg);

int JOINT_PAN_ZERO_CONFIG, JOINT_TILT_ZERO_CONFIG, JOINT_ELBOW_ZERO_CONFIG, JOINT_PADDLE_ZERO_CONFIG;
int JOINT_PAN_MIN, JOINT_TILT_MIN, JOINT_ELBOW_MIN, JOINT_PADDLE_MIN;
int JOINT_PAN_MAX, JOINT_TILT_MAX, JOINT_ELBOW_MAX, JOINT_PADDLE_MAX;
int zero_offset[3];
int joint_min[4];
int joint_max[4];

std::vector<double> thetas(4, -1);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "theta_to_pos");

  ros::NodeHandle n;

  ros::Publisher arm_pub = n.advertise<pongbot::JointGoal>("arm_goal", 1000);
  ros::Publisher wrist_pub = n.advertise<pongbot::WristGoal>("wrist_goal", 1000);
  ros::Subscriber arm_sub = n.subscribe("arm_goal_theta", 1000, updatePosArm);
  ros::Subscriber wrist_sub = n.subscribe("wrist_goal_theta", 1000, updatePosWrist);

  ros::Rate r(100.0);

  n.getParam("joint_pan/zero_config", JOINT_PAN_ZERO_CONFIG);
  n.getParam("joint_tilt/zero_config", JOINT_TILT_ZERO_CONFIG);
  n.getParam("joint_elbow/zero_config", JOINT_ELBOW_ZERO_CONFIG);
  n.getParam("joint_paddle/zero_config", JOINT_PADDLE_ZERO_CONFIG);

  n.getParam("joint_pan/min", JOINT_PAN_MIN);
  n.getParam("joint_tilt/min", JOINT_TILT_MIN);
  n.getParam("joint_elbow/min", JOINT_ELBOW_MIN);
  n.getParam("joint_paddle/min", JOINT_PADDLE_MIN);

  n.getParam("joint_pan/max", JOINT_PAN_MAX);
  n.getParam("joint_tilt/max", JOINT_TILT_MAX);
  n.getParam("joint_elbow/max", JOINT_ELBOW_MAX);
  n.getParam("joint_paddle/max", JOINT_PADDLE_MAX);

  zero_offset[0] = JOINT_PAN_ZERO_CONFIG * DEGREE;
  zero_offset[1] = JOINT_TILT_ZERO_CONFIG * DEGREE;
  zero_offset[2] = JOINT_ELBOW_ZERO_CONFIG * DEGREE;
  zero_offset[3] = JOINT_PADDLE_ZERO_CONFIG * DEGREE;

  joint_min[0] = JOINT_PAN_MIN;
  joint_min[1] = JOINT_TILT_MIN;
  joint_min[2] = JOINT_ELBOW_MIN;
  joint_min[3] = JOINT_PADDLE_MIN;

  joint_max[0] = JOINT_PAN_MAX;
  joint_max[1] = JOINT_TILT_MAX;
  joint_max[2] = JOINT_ELBOW_MAX;
  joint_max[3] = JOINT_PADDLE_MAX;

  while (ros::ok())
  {
    pongbot::JointGoal jointmsg;
    pongbot::WristGoal wristmsg;

    convertTheta(&jointmsg, &wristmsg);

    ROS_INFO("Publishing Goal!");
    arm_pub.publish(jointmsg);
    wrist_pub.publish(wristmsg);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

/** Callback function for getting the arm goal position from subscribed topic
 * */
void updatePosArm(const pongbot::JointGoal::ConstPtr& msg)
{
    for (size_t i=0; i<3; i++)
        thetas[i] = msg->joint_thetas.at(i);
}

/** Callback function for getting the wrist goal position from subscribed topic
 * */
void updatePosWrist(const pongbot::WristGoal::ConstPtr& msg)
{
    thetas[3] = msg->wrist_goal_theta;
}

void convertTheta(pongbot::JointGoal* jointmsg, pongbot::WristGoal* wristmsg)
{
    double tmp;
    for (size_t i=0; i<3; i++) {
        if (i==1) { //the tilt joint rotates the arm with its body rather than its wheel
            tmp = thetas[i] / RADIAN;
            tmp -= zero_offset[i];
            if ((tmp=tmp/DEGREE) > joint_max[i] || tmp < joint_min[i])
                ROS_ERROR("Invalid theta for joint %d", i+1);
            else
                jointmsg->joints[i] = abs(tmp);
        }else {
            tmp = thetas[i] / RADIAN;
            tmp += zero_offset[i];
            if ((tmp=tmp/DEGREE) > joint_max[i] || tmp < joint_min[i])
                ROS_ERROR("Invalid theta for joint %d", i+1);
            else
                jointmsg->joints[i] = tmp;
        }
    }

    //convert wrist
    tmp = thetas[3] / RADIAN;
    tmp += zero_offset[3];
    if ((tmp=tmp/DEGREE) > joint_max[3] || tmp < joint_min[3])
        ROS_ERROR("Invalid theta for wrist joint");
    else
        wristmsg->wrist_goal = tmp;
}