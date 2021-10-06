#!/usr/bin/env python
import sys
import rospy
import actionlib
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q_origin = [0,0,0,0,0,0]
coordinate_end_effector = [0,0,0]
joint_update_flag = False
end_effector_update_flag = False
F_accel = [-0.01,-0.01,-0.01]

urdf_str = rospy.get_param('/robot_description')
ik_solver = IK("base_link", "wrist_3_link", urdf_string=urdf_str)

def move_control(accel):
    global coordinate_end_effector, Q_origin
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 1.0
    g.trajectory.points = []
    if joint_update_flag and end_effector_update_flag:
        q_buffer = Q_origin
        
        for d in range(10):
            coordinate_x = coordinate_end_effector[0] + (F_accel[0]*(d**2))/2
            coordinate_y = coordinate_end_effector[1] + (F_accel[1]*(d**2))/2
            coordinate_z = coordinate_end_effector[2] + (F_accel[2]*(d**2))/2
            #print([coordinate_x,coordinate_y,coordinate_z])            
            sol = ik_solver.get_ik( q_buffer, 
                                    coordinate_x, coordinate_y, coordinate_z,
                                    0.0, 0.0, 0.0, 1.0)
            
            if sol != None:
                #print([sol[0],sol[1],sol[2],sol[3],sol[4],sol[5]])
                q_buffer = [sol[0],sol[1],sol[2],sol[3],sol[4],sol[5]]
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=[sol[0],sol[1],sol[2],sol[3],sol[4],sol[5]], velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 1
        client.send_goal(g)
        """        
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        """

def update_end_effector_position(msg):
    global coordinate_end_effector, end_effector_update_flag
    end_effector_update_flag = True
    x = msg.pose[7].position.x
    y = msg.pose[7].position.y
    z = msg.pose[7].position.z
    coordinate_end_effector = [x,y,z]
    #print(coordinate_end_effector)

def update_joints(msg):
    global Q_origin, joint_update_flag
    joint_update_flag = True
    Q_origin = msg.position

def main():
    global client, F_accel
    if len(sys.argv) > 3:
        print(sys.argv)        
        F_accel[0] = float(sys.argv[1])
        F_accel[1] = float(sys.argv[2])
        F_accel[2] = float(sys.argv[3])
    try:
        rospy.init_node("joints_control", anonymous=True, disable_signals=True)
        rospy.Subscriber("/gazebo/link_states", LinkStates, update_end_effector_position)
        rospy.Subscriber("/joint_states", JointState, update_joints)
        #client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        move_control(F_accel)

        #rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
