#!/usr/bin/env python

import rospy
import modern_robotics as mr
import numpy as np
import itertools as it
from pongbot.msg import HomoTransformMatrix

class PongbotTrajectoryGeneration(object):

    def __init__(self):
        self.traj_pub = rospy.Publisher("end_effector_transform", HomoTransformMatrix, queue_size=1)
                                            

        self.Xstart = np.array([[0, 0, 1, 379.93],
                          [1, 0, 0,      0],
                          [0, 1, 0, 84.455],
                          [0, 0, 0,      1]])

        # grabbed this from forward kinematics of a dummy publish
        # thetas should be PI/2, PI/2, PI/6
        self.Xend = np.array([[-0.999994, 0.001599, 0.003103, 0.546499],
                          [0.003491, 0.458181, 0.888852,      156.559684],
                          [0, 0.888857, -0.458184, -124.113252],
                          [0, 0, 0,      1]])
        self.N = 50
        self.Tf = 10
        self.method = 5
        self.trajectory = None
        self.count = -1
        self.swap = False

        self.timer = rospy.Timer(rospy.Duration(1/5.0), self.publish_trajectory)

    def publish_trajectory(self, event):
        # we want to publish the full trajectory before recalculating a new one
        if self.count >= self.N or self.count < 0:
            rospy.loginfo("Calculating new trajectory")
            if self.swap:
                self.trajectory = mr.ScrewTrajectory(self.Xend, self.Xstart, self.Tf, self.N, self.method)
            else:
                self.trajectory = mr.ScrewTrajectory(self.Xstart, self.Xend, self.Tf, self.N, self.method)
            #np.set_printoptions(precision=3, suppress=True)
            #print(self.trajectory[0])
            self.count = 0
            self.swap = not self.swap
        else:
            config = []
            for row in range(len(self.trajectory[self.count])):
                for col in self.trajectory[self.count][row]:
                    config.append(col)
            self.traj_pub.publish(config)
            self.count += 1

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('trajectory_generation', anonymous=False)
    # Create the object
    node = PongbotTrajectoryGeneration()
    # Keep it spinning to keep the node alive
    rospy.spin()

