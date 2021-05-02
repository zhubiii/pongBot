#!/usr/bin/env python

import rospy
import modern_robotics as mr
import numpy as np
import itertools as it
from pongbot.msg import JointGoal
from pongbot.msg import HomoTransformMatrix

class PongbotFkin(object):

    def __init__(self):
        self.config_pub = rospy.Publisher("pongbot_config", HomoTransformMatrix, queue_size=1)
        self.joints_sub = rospy.Subscriber("joint_pos", JointGoal,
                                            self.updatePos)

        self.RADIAN = np.pi/180.0
        self.DEGREE = .29
        self.joint_pan_zero_config = rospy.get_param("joint_pan/zero_config")
        self.joint_tilt_zero_config = rospy.get_param("joint_tilt/zero_config")
        self.joint_elbow_zero_config = rospy.get_param("joint_elbow/zero_config")
        self.zero_offset = np.array([self.joint_pan_zero_config*self.DEGREE,
                                     self.joint_tilt_zero_config*self.DEGREE,
                                     self.joint_elbow_zero_config*self.DEGREE])
        self.M = np.array([[0, 0, 1, 379.93],
                          [1, 0, 0,      0],
                          [0, 1, 0, 84.455],
                          [0, 0, 0,      1]])
        self.Blist = np.array([[0, 1, 0, 379.93, 0, 0],
                              [-1,0, 0, 0, 379.93, 65.745],
                              [-1,0, 0, 0, 235.6, 25]]).T
        self.thetalist = np.array([-1.0,-1.0,-1.0])

        # homogeneous transform matrix from fkin
        self.T = None
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_fkin)

    def updatePos(self, msg):
        for joint, i in it.izip(msg.joints, range(len(msg.joints)-1)): #exclude wrist for now
            #if i==1:
                #self.thetalist[i] = ((joint*self.DEGREE) + self.zero_offset[i]) * self.RADIAN
            #else:
            self.thetalist[i] = ((joint*self.DEGREE) - self.zero_offset[i]) * self.RADIAN

    def publish_fkin(self, event):
        print(self.thetalist)
        self.T = mr.FKinBody(self.M, self.Blist, self.thetalist)
        np.set_printoptions(precision=3, suppress=True)
        print(self.T)
        #self.config_pub.publish(self.T)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('pongbot_fkin', anonymous=False)
    # Create the object
    node = PongbotFkin()
    # Keep it spinning to keep the node alive
    rospy.spin()
