#!/usr/bin/env python
'''
Description: Action handler for sending trajectories

Created on May 12, 2014

@author: Mark Cutler
@email: markjcutler@gmail.com
'''
import rospy
import actionlib
import numpy as np

# ros imports
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

# custom imports
from path_planner.msg import *
from quad_control.msg import Goal
from aclpy import utils

def lin_interp(pos, vel, acc, jerk, t):
    pos_current = Vector3()
    vel_current = Vector3()
    acc_current = Vector3()
    jerk_current = Vector3()
    
    pos_current.x = t*pos[1].x + (1.0-t)*pos[0].x
    vel_current.x = t*vel[1].x + (1.0-t)*vel[0].x
    acc_current.x = t*acc[1].x + (1.0-t)*acc[0].x
    jerk_current.x = t*jerk[1].x + (1.0-t)*jerk[0].x
    
    pos_current.y = t*pos[1].y + (1.0-t)*pos[0].y
    vel_current.y = t*vel[1].y + (1.0-t)*vel[0].y
    acc_current.y = t*acc[1].y + (1.0-t)*acc[0].y
    jerk_current.y = t*jerk[1].y + (1.0-t)*jerk[0].y
    
    pos_current.z = t*pos[1].z + (1.0-t)*pos[0].z
    vel_current.z = t*vel[1].z + (1.0-t)*vel[0].z
    acc_current.z = t*acc[1].z + (1.0-t)*acc[0].z
    jerk_current.z = t*jerk[1].z + (1.0-t)*jerk[0].z
    return pos_current, vel_current, acc_current, jerk_current

class BroadcastTrajectoryServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('broadcast_trajectory',
                                                   BroadcastTrajectoryAction,
                                                   self.broadcast,
                                                   False)
        self.goal_pub = rospy.Publisher('goal', Goal, queue_size=1)
        self.server.start()
        self.current_pose = Pose()
        self.poseSub = rospy.Subscriber("pose", PoseStamped, self.poseCB)
        self.goalSub = rospy.Subscriber("goal", Goal, self.goalCB)
        self.current_goal = Goal()
        self.name = rospy.get_namespace()
        self.name = self.name[1:-1]  # remove initial and trailing slashes

    def goalCB(self, data):
        self.current_goal = data

    def poseCB(self, data):
        self.current_pose = data.pose

# -----------------------------------------------------------------------------
# broadcast a trajectory
# -----------------------------------------------------------------------------
    def broadcast(self, goal):
        print "starting trajectory broadcast"

        fb = BroadcastTrajectoryFeedback()
        start_time = rospy.get_time()
        i = 0
        K = len(goal.trajectory.pos)

        qg = Goal()
        qg.header.frame_id = self.name
        qg.dyaw = 0.0
        qg.upright = True
        qg.waypointType = 0
        qg.yaw = self.current_goal.yaw
        starting_goal = self.current_goal.pos
        #print goal.trajectory.pos
        r = rospy.Rate(200) # 200hz
        #print goal.trajectory.acc
        while i < K-1:
            current_time = rospy.get_time() - start_time
    
            t0 = goal.trajectory.pos[i].header.stamp.to_sec()
            t1 = goal.trajectory.pos[i+1].header.stamp.to_sec()
            t = (current_time-t0)/(t1-t0)
            pos = [goal.trajectory.pos[i].vector, goal.trajectory.pos[i+1].vector]
            vel = [goal.trajectory.vel[i].vector, goal.trajectory.vel[i+1].vector]
            acc = [goal.trajectory.acc[i].vector, goal.trajectory.acc[i+1].vector]
            jerk = [goal.trajectory.jerk[i].vector, goal.trajectory.jerk[i+1].vector]
            qg.pos, qg.vel, qg.accel, qg.jerk = lin_interp(pos, vel, acc, jerk, t)
#            qg.pos = goal.trajectory.pos[i].vector
#            qg.vel = goal.trajectory.vel[i].vector
#            qg.accel = goal.trajectory.acc[i].vector
            #qg.jerk = goal.trajectory.jerk[i].vector
            #qg.jerk.x = 0
            #qg.jerk.y = 0
            #qg.jerk.z = 0
            if goal.relative_starting_coordinates:
                qg.pos.x += starting_goal.x
                qg.pos.y += starting_goal.y
                qg.pos.z += starting_goal.z

            self.goal_pub.publish(qg)
            
            fb.percent_complete = i/K*100.0
            self.server.publish_feedback(fb) # publish % complete
            if t1 <= current_time:
                i += 1

            r.sleep()
            
        qg = Goal() # make sure final vel, acc, and jerk are zero
        qg.pos = goal.trajectory.pos[-1].vector
        self.goal_pub.publish(qg)
        fb.percent_complete = 100.0
        self.server.publish_feedback(fb) # publish % complete
            
        self.server.set_succeeded()


if __name__ == '__main__':
    ns = rospy.get_namespace()
    try:
        rospy.init_node('broadcast_trajectory_server')

        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=BQ01 $ rosrun path_planner broadcast_trajectory.py")
        else:
            argv = rospy.myargv(argv=sys.argv)
            c = BroadcastTrajectoryServer()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
