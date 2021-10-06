#!/usr/bin/env python
from __future__ import division

import rospy
import actionlib
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q_origin = [2.2,0,-1.57,1,2,3]

T=20

def move_sinewave():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []

    for i in range(4*T):
        r = round(np.sin(((i+1)/T)*2*np.pi),2)
        #print([r*np.pi,(-np.pi/2)*(1+r) ,r*2.8,(-np.pi/2)*(1+r),r*np.pi,r*np.pi])
        #print(((i+1)/T)*2*180)
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=[r*np.pi,(-np.pi/2)*(1+r) ,r*2.8,(-np.pi/2)*(1+r),r*np.pi,r*np.pi], velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("sine_wave_joints", anonymous=True, disable_signals=True)
        #client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        move_sinewave()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

