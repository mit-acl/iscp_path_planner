#!/usr/bin/env python
import rospy
import numpy as np
from acl_msgs.msg import Waypoint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# custom imports
from acl_msgs.msg import QuadGoal as Goal
from acl_msgs.srv import MultiVehWaypoint
from acl_msgs.srv import MultiVehTakeoffLand
from acl_msgs.srv import MultiVehPreCompute
from acl_msgs.srv import GenPath
from acl_msgs.msg import QuadPathArray as PathArray
from acl_msgs.msg import QuadPath as Path
import actionlib

from aclpy import utils

TAKEOFF = 1
DISABLE = 2
RESET_INTEGRATORS = 4
ATTITUDE = 5

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
def initMarker(m):
    m.header.frame_id = "/world"
    m.header.stamp = rospy.Time.now()
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.0

    m.ns = "RAVEN_path"
#    m.id = 0
#    for letter in str(name):
#        m.id += ord(letter)  # cheap way to get a unique marker id

    m.id = 345

    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1
    m.scale.x = 0.045
    m.scale.y = 0.03
    m.scale.z = 0.03
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0

    m.lifetime = rospy.Duration(500)
    return m

def sendPath(path, gz=None):

    pathmarker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    initMarker(pathmarker)

    for i in range(len(path.pos)):
        p = Point()
        p.x = path.pos[i].vector.x
        p.y = path.pos[i].vector.y
        if gz is not None:
            path.pos[i].vector.z = gz
        p.z = path.pos[i].vector.z
        pathmarker.points.append(p)

#    print name
    client = actionlib.SimpleActionClient('broadcast_trajectory',
                                          BroadcastTrajectoryAction)

    print "waiting for broadcast_trajectory server"
    client.wait_for_server()
    print "got broadcast_trajectory server"
    goal = BroadcastTrajectoryGoal(path, False)

    return client, goal, pathmarker

def isValid(p, z=1.0):
    
    '''ACL spec'''
    x_max = 3.0 #2.5
    x_min = -7.5
    y_max = 2.9
    y_min = -7.3
    z_max = 2.0
    z_min = -0.5
    x_cor = -2.5
    y_cor = -1.8
    pole_x_min = -3.9
    pole_x_max = -2.6
    pole_y_min = -5.0
    pole_y_max = -3.7
    
    '''not too close to other points'''
    valid_point = 1
    
    # not to hight
    if (z > z_max or z < z_min):
        valid_point = 0
    
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

class Waypoint2Goal():

    def __init__(self):
        self.waypoint = Waypoint()
        self.pose = Pose()
        self.twist = Twist()
        self.recent_goal = Goal()
        self.pub_goal = rospy.Publisher('goal', Goal, queue_size=1)
        self.pub_marker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True, queue_size=1)
        self.name = rospy.get_namespace()
        self.name = self.name[1:-1]  # remove initial and trailing slashes
        
        rospy.loginfo('Waiting for path planner service')
        rospy.wait_for_service('/gen_path')
        self.genPath = rospy.ServiceProxy('/gen_path', GenPath)
        rospy.loginfo('Got path planner service')
    
        rospy.loginfo('Waiting for simple path planner service')
        rospy.wait_for_service('/simple_gen_path')
        self.genSimplePath = rospy.ServiceProxy('/simple_gen_path', GenSimplePath)
        rospy.loginfo('Got simple path planner service')
        

    def waypointCB(self, data):
        self.waypoint = data
        print self.waypoint
        print self.name
        if self.waypoint.header.frame_id == self.name:
            goal = Goal()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.name
            goal.pos.x = self.waypoint.goal_pose.position.x
            goal.pos.y = self.waypoint.goal_pose.position.y
            goal.pos.z = self.waypoint.goal_pose.position.z
            goal.yaw = utils.quat2yaw(self.waypoint.goal_pose.orientation)
            if self.waypoint.takeoff:
                goal.waypointType = TAKEOFF
                self.pub_goal.publish(goal)
            elif self.waypoint.land:
                goal.waypointType = DISABLE
                self.pub_goal.publish(goal)
            else:
                goal.waypointType = 0
                
                if self.waypoint.acceleration > 0.:
                    # use the simple path planner
                    if isValid([goal.pos.x, goal.pos.y, goal.pos.z]):

                        p1v = Vector3()
                        p1v.x = goal.pos.x
                        p1v.y = goal.pos.y
                        p1v.z = goal.pos.z
                        p0v = Vector3()
                        p0v.x = self.recent_goal.pos.x
                        p0v.y = self.recent_goal.pos.y
                        p0v.z = self.recent_goal.pos.z
                    
                        resp = self.genSimplePath(self.waypoint.velocity, 
                                                  self.waypoint.acceleration, 
                                                  p0v, p1v)
                        client, goal, marker = sendPath(resp.trajectory)
                    
                        # broadcast path marker for visualization purposes
                        ma = MarkerArray()
                        ma.markers.append(marker)
                    
                        self.pub_marker.publish(ma)
                    
                        client.send_goal(goal)
                        client.wait_for_result()                    
                    else:
                        rospy.logerr('Infeasible final goal')       
                        
                else:
                    # run complex path planner
                    if isValid([goal.pos.x, goal.pos.y]):
                        if utils.distance2(goal.pos.x, goal.pos.y, 
                                           self.recent_goal.pos.x, 
                                           self.recent_goal.pos.y) > 0.2:                        
                            p0 = np.array([self.recent_goal.pos.x,
                                           self.recent_goal.pos.y])
                            p1 = np.array([goal.pos.x, goal.pos.y])
                            v0 = np.zeros((1, 2))
                            v1 = np.zeros((1, 2))
                            a0 = np.zeros((1, 2))
                            a1 = np.zeros((1, 2))
                        
                            N = 1
                            numD = 2
                            h = 0.25
                            T = 5
                            R = 0.8
                        
                            # temporary static obstacle: THIS IS DISABLED IN PATHPLANNER RIGHT NOW
                            obs = np.array([0, -2])
                        
                            p0v = numpyArray2DToVector3Array(p0, numD)
                            v0v = numpyArray2DToVector3Array(v0, numD)
                            a0v = numpyArray2DToVector3Array(a0, numD)
                            p1v = numpyArray2DToVector3Array(p1, numD)
                            v1v = numpyArray2DToVector3Array(v1, numD)
                            a1v = numpyArray2DToVector3Array(a1, numD)
                            obsv = numpyArray2DToVector3Array(obs, numD)                   
                        
                            resp = self.genPath(N, numD, T, h, R, p0v, v0v, a0v, p1v, v1v, a1v, obsv)
                            if resp.converged:
                                client0, goal0, marker0 = sendPath(resp.trajectories[0], self.recent_goal.pos.z)
                        
                                # broadcast path marker for visualization purposes
                                ma = MarkerArray()
                                ma.markers.append(marker0)
                        
                                self.pub_marker.publish(ma)
                        
                                client0.send_goal(goal0)
                                client0.wait_for_result()
                            else:
                                rospy.logerr('Path Planner did not converge')
                        else:
                            self.pub_goal.publish(goal)
                    else:
                        rospy.logerr('Infeasible final goal')                    
                    

    def poseCB(self, data):
        self.pose = data.pose

    def twistCB(self, data):
        self.twist = data.twist

    def goalCB(self, data):
        self.recent_goal = data


if __name__ == '__main__':
    ns = rospy.get_namespace()
    try:
        rospy.init_node('waypoint2goal')

        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=BQ01 $ rosrun package file.py")
        else:
            c = Waypoint2Goal()
            rospy.Subscriber("quad_waypoint", Waypoint, c.waypointCB)
            rospy.Subscriber("pose", PoseStamped, c.poseCB)
            rospy.Subscriber("vel", TwistStamped, c.twistCB)
            rospy.Subscriber("goal", Goal, c.goalCB)
            rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
