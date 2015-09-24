#!/usr/bin/env python
'''
Description: Simple service demo for multiple quads

Created on October 3, 2014

@author: Mark Cutler
@email: markjcutler@gmail.com
'''
import roslib
roslib.load_manifest('iscp_path_planner')
import rospkg
import rospy
import numpy as np

# ros imports
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

from acl_msgs.srv import MultiVehWaypoint
from acl_msgs.srv import MultiVehTakeoffLand
from acl_msgs.srv import MultiVehPreCompute

def send_waypoint(final_positions):
    rospy.wait_for_service('multi_vehicle_waypoint')
    try:
        s = rospy.ServiceProxy('multi_vehicle_waypoint', MultiVehWaypoint)
        print final_positions
        resp = s(final_positions)
        return resp.converged
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def send_pre_compute(file_name):
    rospy.wait_for_service('multi_vehicle_pre_compute')
    try:
        s = rospy.ServiceProxy('multi_vehicle_pre_compute', MultiVehPreCompute)
        print file_name
        resp = s(file_name)
        return resp.converged
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def send_takeoff_land(takeoff, use_current_pose=False, initial_pose=[]):
    rospy.wait_for_service('multi_vehicle_takeoff_land')
    try:
        s = rospy.ServiceProxy('multi_vehicle_takeoff_land', MultiVehTakeoffLand)
        resp = s(takeoff, use_current_pose, initial_pose)
        return resp.done
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# vehicles = 01, 02, 03, 04
#p = np.array([[[0.8, -2.5],[-0.7, -1.0],[-0.7, -4.0],[-2.2, -2.5]],
#                [[-2.2, -2.5],[-0.7, -4.0],[-0.7, -1.0],[0.8, -2.5]],
#                [[0.0, -1.5],[-0.0, -3.5],[-0.0, -0.5],[-0.0, -2.5]],
#                [[-5.0, -3.0],[-5.0, -5.0],[-5.0, -2.0],[-5.0, -4.0]],
#                [[-5.0, -4.0],[-6.0, -5.0],[-6.0, -4.0],[-5.0, -5.0]],
#                [[0.0, -5.0],[-1.0, -5.0],[0.0, -4.0],[-1.0, -4.0]],
#                [[0.0, -5.0],[-5.0, -5.0],[0.0, -4.0],[-5.0, -4.0]],
#                [[-5.0, -5.0],[0.0, -5.0],[-5.0, -4.0],[0.0, -4.0]],
#                [[-0.5, 0.0],[-1.5, -1.0],[-0.5, -1.0],[-1.5, -0.0]],
#                [[-4.0, -2.2],[-5.0, -3.2],[-4.0, -3.2],[-5.0, -2.2]],
#                [[-0.7, -4.3], [-5.3, -4.3], [-3.2,-6.3], [-3.2,-2.2]],
#                [[-5.3, -4.3], [-0.7, -4.3], [-3.2,-2.2], [-3.2,-6.3]],
#                [[0.0, 0.3],[-6.0, -4.7],[-4.0, -6.1],[-5.9, -5.7]],
#                [[-5.5, -3.7],[-0.8, -3.8],[-3.3, -3.0],[-2.0, -5.0]],
#                [[-1.5, 0.0],[-1.5, -5.0],[0.0, -0.0],[0.0, -5.0]]])

p = np.array([[[0.8, -2.5],[-0.7, -1.0],[-0.7, -4.0],[-2.2, -2.5]],
               [[-2.2, -2.51],[-0.7, -4.01],[-0.7, -1.01],[0.8, -2.51]],
               [[0.8, -2.51],[-2.2, -2.51],[-0.7, -4.01],[-0.7, -1.01]],
               [[0.8, -2.51],[-0.7, -2.51],[-1.5, -4.01],[-1.5, -1.01]],                                                           
               [[-1.5, -4.01],[-0.7, -2.51],[-1.5, -1.01],[0.8, -2.51]],
               [[0.7, -1.01],[-0.5, -2.51],[0.7, -4.01],[-1.7, -2.51]],
               [[-0.7, -4.51],[-0.7, -3],[-0.7, -1.5],[-0.7, 0]],
               [[-0.7, -1.5],[-0.7, -3],[-0.7, 0],[-0.7, -4.51]],
               [[-1, -1.5],[-1, -3],[1, -1.5],[1, -3]],
               [[1, -3.75],[1, -1],[-1, -3.75],[-1, -1]],
               [[1, -1],[1, -3.75],[-1, -1],[-1, -3.75]],
               [[0, -3],[0, -1.5],[0, 0],[0, -4.5]]])

rospack = rospkg.RosPack()
package_path = rospack.get_path('iscp_path_planner')
path = package_path + '/trajectories/'
trajectories = ['traj1.txt', 'traj2.txt', 'traj3.txt', 'traj4.txt', 'traj5.txt', 
                'traj6.txt', 'traj7.txt', 'traj8.txt', 'traj9.txt', 'traj10.txt', 'traj11.txt']

z = 0.75
initial_pose = [Pose() for i in range(4)]
initial_pose[0].position.x = 0.7+.3
initial_pose[0].position.y = -2.5-1
initial_pose[0].position.z = z
initial_pose[1].position.x = -0.7+.3
initial_pose[1].position.y = -1-1
initial_pose[1].position.z = z
initial_pose[2].position.x = -0.7+.3
initial_pose[2].position.y = -4-1
initial_pose[2].position.z = z
initial_pose[3].position.x = -2.2+.3
initial_pose[3].position.y = -2.5-1
initial_pose[3].position.z = z


send_takeoff_land(True, False, initial_pose)
rospy.sleep(5)

for traj in trajectories:
    print send_pre_compute(path+traj)
    rospy.sleep(1)

#for j in range(p.shape[0]):
#    final_positions = []
#    for i in range(4):
#        pf = Vector3()
#        pf.x = p[j,i,0]
#        pf.y = p[j,i,1]
#        final_positions.append(pf)
#    print send_waypoint(final_positions)
#    rospy.sleep(1)
    
send_takeoff_land(False)
     
