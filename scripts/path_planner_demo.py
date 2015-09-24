#!/usr/bin/env python
import roslib
roslib.load_manifest('iscp_path_planner')
import rospy
import sys
import numpy as np
import actionlib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from acl_msgs.msg import *
from acl_msgs.msg import Waypoint
from acl_msgs.msg import QuadGoal as Goal
from acl_msgs.srv import MultiVehWaypoint
from acl_msgs.srv import MultiVehTakeoffLand
from acl_msgs.srv import MultiVehPreCompute
from acl_msgs.srv import GenPath
from acl_msgs.msg import QuadPathArray as PathArray
from acl_msgs.msg import QuadPath as Path
from iscp_path_planner import plot_trajectories
#from quad_control.msg import Goal
from geometry_msgs.msg import PoseStamped


def isValid(p):
    
    '''ACL spec'''
    x_max = 1.1
    x_min = -7.5
    y_max = 1.0
    y_min = -6.9
    x_cor = -2.0
    y_cor = -1.8
    pole_x_min = -3.9
    pole_x_max = -2.6
    pole_y_min = -5.0
    pole_y_max = -3.7
    
    '''not too close to other points'''
    valid_point = 1
    
    # inside the room
    if (p[0] > x_max or p[0] < x_min or
        p[1] > y_max or p[1] < y_min ):
        valid_point = 0
        
    '''corner'''
    if (p[0] < x_cor and p[1] >y_cor):
        valid_point = 0
        
    '''pole'''
    if (p[0] < pole_x_max and p[0] > pole_x_min and 
        p[1] <pole_y_max and p[1] >pole_y_min):
        valid_point = 0
        
    return valid_point

def numpyArray2DToVector3Array(na, numD):
    vec_array = []
    if len(na.shape) == 1:
        vec = Vector3()
        vec.x = na[0]
        vec.y = na[1]
        if numD > 2:
            vec.z = na[2]
        vec_array.append(vec)
    else:
        for i in range(na.shape[0]):
            vec = Vector3()
            if numD > 0:
                vec.x = na[i, 0]
            if numD > 1:
                vec.y = na[i, 1]
            if numD > 2:
                vec.z = na[i, 2]
            vec_array.append(vec)

    return vec_array


# setup goal marker for visualization in rviz
def initMarker(name, m):
    m.header.frame_id = "/world"
    m.header.stamp = rospy.Time.now()
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.0

    m.ns = "RAVEN_path"
    m.id = 0
    for letter in str(name):
        m.id += ord(letter)  # cheap way to get a unique marker id

    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1
    m.scale.x = 0.03
    m.scale.y = 0.03
    m.scale.z = 0.03
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0

    if name == 'BQ01':
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
    if name == 'BQ02':
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
    if name == 'BQ04':
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0

    m.lifetime = rospy.Duration(10)
    return m


def sendPath(name, path, gz):

    pathmarker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    initMarker(name, pathmarker)

    for i in range(len(path.pos)):
        p = Point()
        p.x = path.pos[i].vector.x
        p.y = path.pos[i].vector.y
        path.pos[i].vector.z = gz
        p.z = path.pos[i].vector.z
        pathmarker.points.append(p)

    print name
    client = actionlib.SimpleActionClient('/' + name + '/broadcast_trajectory',
                                          BroadcastTrajectoryAction)

    print "waiting for broadcast_trajectory server"
    client.wait_for_server()
    print "got broadcast_trajectory server"
    goal = BroadcastTrajectoryGoal(path, False)

    return client, goal, pathmarker

def main(name, xf, yf):
    
    # get the current goal of the vehicle
    print 'Waiting for goal message from vehicle ' + name
    goal = rospy.wait_for_message('/'+name+'/pose', PoseStamped)
    print 'Received goal message:'
    print goal
    
    gx = goal.pose.position.x
    gy = goal.pose.position.y
    gz = goal.pose.position.z
    
    # check that your points are valid
    if not isValid([gx, gy]):
        rospy.logerr('Infeasible final goal')
        return -1
    
    # check that your points are valid
    if not isValid([xf, yf]):
        rospy.logerr('Infeasible final goal')
        return -1
    

    rospy.wait_for_service('/gen_path')
    genPath = rospy.ServiceProxy('/gen_path', GenPath)

    p0 = np.array([gx,  gy])
    p1 = np.array([xf, yf])
    v0 = np.zeros((1, 2))
    v1 = np.zeros((1, 2))
    a0 = np.zeros((1, 2))
    a1 = np.zeros((1, 2))

    N = 1
    numD = 2
    h = 0.025
    T = 1
    R = 0.8
    
    p0v = numpyArray2DToVector3Array(p0, numD)
    v0v = numpyArray2DToVector3Array(v0, numD)
    a0v = numpyArray2DToVector3Array(a0, numD)
    p1v = numpyArray2DToVector3Array(p1, numD)
    v1v = numpyArray2DToVector3Array(v1, numD)
    a1v = numpyArray2DToVector3Array(a1, numD)

    resp = genPath(N, numD, T, h, R, p0v, v0v, a0v, p1v, v1v, a1v)

    #plot_trajectories.plot(resp.trajectories, False)

    client0, goal0, marker0 = sendPath(name, resp.trajectories[0], gz)

    # broadcast path marker for visualization purposes
    ma = MarkerArray()
    ma.markers.append(marker0)
    #ma.markers.append(marker1)
    pubMarker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True, queue_size=1)
    pubMarker.publish(ma)

    #ans=raw_input('accept solution?')

    #if ans == 'y':
    client0.send_goal(goal0)
    client0.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('send_path')
    
    args = rospy.myargv(argv=sys.argv)
    if len(args) == 4:
        name = args[1]
        xf = float(args[2])
        yf = float(args[3])
        main(name,xf,yf)
    else:
        rospy.logerr('Not right input arguments. Should be `name xf yf`')
    
