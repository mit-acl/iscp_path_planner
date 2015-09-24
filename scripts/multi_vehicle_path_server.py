#!/usr/bin/env python
'''
Description: Simple service for keeping track of current goals of 
multiple vehicles and calling path planner for them

Created on October 3, 2014

@author: Mark Cutler
@email: markjcutler@gmail.com
'''
import roslib
roslib.load_manifest('iscp_path_planner')
import rospy
import numpy as np

# ros imports
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import actionlib

# custom imports
from acl_msgs.msg import Trajectory
from acl_msgs.msg import *
from acl_msgs.msg import Waypoint
from acl_msgs.msg import QuadGoal as Goal
from acl_msgs.srv import MultiVehWaypoint
from acl_msgs.srv import MultiVehTakeoffLand
from acl_msgs.srv import MultiVehPreCompute
from acl_msgs.srv import GenPath
from acl_msgs.msg import QuadPathArray as PathArray
from acl_msgs.msg import QuadPath as Path

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
def initMarker(m, i):
    m.header.frame_id = "/world"
    m.header.stamp = rospy.Time.now()
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.0

    m.ns = "RAVEN_path"
#    m.id = 0
#    for letter in str(name):
#        m.id += ord(letter)  # cheap way to get a unique marker id

    m.id = 345 + i

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
    
    if i == 0:
        m.color.r = 1.0
    elif i == 1:
        m.color.r = 0
        m.color.g = 1.0
    elif i == 2:
        m.color.r = 1.0
        m.color.b = 1.0
    elif i == 3:
        m.color.g = 1.0

    m.lifetime = rospy.Duration(10)
    return m

def sendPath(name, path, gz, ii):

    pathmarker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    initMarker(pathmarker, ii)

    for i in range(len(path.pos)):
        p = Point()
        p.x = path.pos[i].vector.x
        p.y = path.pos[i].vector.y
        path.pos[i].vector.z = gz
        p.z = path.pos[i].vector.z
        pathmarker.points.append(p)

#    print name
    client = actionlib.SimpleActionClient(name+'/broadcast_trajectory',
                                          BroadcastTrajectoryAction)

    print "waiting for broadcast_trajectory server"
    client.wait_for_server()
    print "got broadcast_trajectory server"
    goal = BroadcastTrajectoryGoal(path, False)

    return client, goal, pathmarker

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

def readFromFile (filename):
    f = open ( filename , 'r')
    data = []

    for line in f:
        #print line
        data_line = []
        for x in line.split('\t'):
            if x!='\n':
                data_line.append(float(x))
        data.append(data_line)

    #print data
    return data

class MultiVehicleWaypointServer:

    def __init__(self, veh):
        self.veh = veh
        self.N = len(self.veh)
        self.numD = 2
        self.current_goal = [Goal() for i in range(len(self.veh))]
        self.current_pose = [Pose() for i in range(len(self.veh))]

        self.pub_goal = []
        for i in range(self.N):
            self.goalSub = rospy.Subscriber(self.veh[i] + "/goal", Goal, self.goalCB)
            self.poseSub = rospy.Subscriber(self.veh[i] + "/pose", PoseStamped, self.poseCB)
            self.pub_goal.append(rospy.Publisher(self.veh[i] + '/goal', Goal, queue_size=1))
        self.pub_marker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True, queue_size=1)
        self.pub_path_array = rospy.Publisher('/projector_paths', PathArray, latch=True, queue_size=1)


        self.server = rospy.Service('multi_vehicle_waypoint', MultiVehWaypoint, self.multi_vehicle_waypoint)
        self.server2 = rospy.Service('multi_vehicle_takeoff_land', MultiVehTakeoffLand, self.multi_vehicle_takeoff_land)
        self.server3 = rospy.Service('multi_vehicle_pre_compute', MultiVehPreCompute, self.multi_vehicle_pre_compute)
        rospy.wait_for_service('/gen_path')
        self.genPath = rospy.ServiceProxy('/gen_path', GenPath)
        


    def goalCB(self, data):
        for i in range(self.N):
            if data.header.frame_id == self.veh[i]:
                self.current_goal[i] = data
               # print self.current_goal[i].pos.y
               
    def poseCB(self, data):
        for i in range(self.N):
            if data.header.frame_id == self.veh[i]:
                self.current_pose[i] = data.pose
               # print self.current_goal[i].pos.y

    def multi_vehicle_takeoff_land(self, req):
        if req.takeoff:
            for i in range(self.N):
                goal = Goal()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = self.veh[i]
                if req.use_current_pose:
                    goal.pos.x = self.current_pose[i].position.x
                    goal.pos.y = self.current_pose[i].position.y
                    goal.pos.z = 0.5
                else:
                    goal.pos.x = req.initial_pose[i].position.x
                    goal.pos.y = req.initial_pose[i].position.y
                    goal.pos.z = req.initial_pose[i].position.z
                goal.yaw = utils.quat2yaw(self.current_pose[i].orientation)
                goal.waypointType = TAKEOFF
                self.pub_goal[i].publish(goal)
            return True
        else: # landing
            while (1):
                landed = True
                for i in range(self.N):
                    goal = Goal()
                    goal.header.stamp = rospy.Time.now()
                    goal.header.frame_id = self.veh[i]
                    goal.pos.x = self.current_goal[i].pos.x
                    goal.pos.y = self.current_goal[i].pos.y
                    if self.current_pose[i].position.z > 0.4:
                       # fast landing
                       goal.pos.z = utils.saturate(self.current_goal[i].pos.z - 0.01,
                                                   2.0, -0.1)
                    else:
                        # slow landing
                        goal.pos.z = utils.saturate(self.current_goal[i].pos.z - 0.0025,
                                                    2.0, -0.1)
                    goal.yaw = self.current_goal[i].yaw
                    goal.waypointType = 0
                    if goal.pos.z <= -0.1:
                        goal.waypointType = DISABLE
                    else:
                        landed = False
                    self.pub_goal[i].publish(goal)
                    rospy.sleep(0.01) # ~100 Hz
                 
                # once all the quads have landed, return    
                if landed:
                    break
                
            return True
                    
    def multi_vehicle_pre_compute(self, req):
        
        data = readFromFile(req.file_name)
        trajectories = [Trajectory() for i in range(len(self.veh))]
        for data_line in data:
            cnt = 1
            for i in range(self.N):
                posLine = Vector3Stamped()
                posLine.header.stamp = rospy.Time(data_line[0])
                posLine.vector.x = data_line[cnt]
                cnt += 1
                posLine.vector.y = data_line[cnt]
                cnt += 1
                posLine.vector.z = 0
                trajectories[i].pos.append(posLine)
            
                velLine = Vector3Stamped()
                velLine.header = posLine.header
                velLine.vector.x = data_line[cnt]
                cnt += 1
                velLine.vector.y = data_line[cnt]
                cnt += 1
                velLine.vector.z = 0
                trajectories[i].vel.append(velLine)
            
                accLine = Vector3Stamped()
                accLine.header = posLine.header
                accLine.vector.x = data_line[cnt]
                cnt += 1
                accLine.vector.y = data_line[cnt]
                cnt += 1
                accLine.vector.z = 0
                trajectories[i].acc.append(accLine)
                
                jerkLine = Vector3Stamped()
                jerkLine.header = posLine.header
                jerkLine.vector.x = data_line[cnt]
                cnt += 1
                jerkLine.vector.y = data_line[cnt]
                cnt += 1
                jerkLine.vector.z = 0
                trajectories[i].jerk.append(jerkLine)
        
        ma = MarkerArray()
        client_array = []
        goal_array = []
        pa = PathArray()
        for i in range(self.N):
            client, goal, marker = sendPath(self.veh[i], trajectories[i], self.current_goal[i].pos.z, i)
            # broadcast path marker for visualization purposes
            ma.markers.append(marker)
            client_array.append(client)
            goal_array.append(goal)
            
            path = Path()
            path.header.frame_id = self.veh[i]
            for j in range(len(trajectories[i].pos)):
                path.x.append(trajectories[i].pos[j].vector.x)
                path.y.append(trajectories[i].pos[j].vector.y)
            pa.path_array.append(path)
            
                
        
        self.pub_marker.publish(ma)
        self.pub_path_array.publish(pa)
        for i in range(self.N):
            client_array[i].send_goal(goal_array[i])
            
        client_array[self.N-1].wait_for_result()
        return True            

    def multi_vehicle_waypoint(self, req):
        points_valid = True

        p0 = np.zeros((self.N, self.numD))
        v0 = np.zeros((self.N, self.numD))
        a0 = np.zeros((self.N, self.numD))
        p1 = np.zeros((self.N, self.numD))
        v1 = np.zeros((self.N, self.numD))
        a1 = np.zeros((self.N, self.numD))

        h = 0.25
        T = 10
        R = 0.8

        for i in range(self.N):
            print i
            print req
            if not isValid([req.final_positions[i].x,
                            req.final_positions[i].y]):
                print req.final_positions[i].x
                print req.final_positions[i].y
                rospy.logerr('Final positions not valid!')
                return False
            p0[i,:] = np.array([self.current_goal[i].pos.x,
                           self.current_goal[i].pos.y])
            p1[i,:] = np.array([req.final_positions[i].x,
                                req.final_positions[i].y])
            
        # temporary static obstacle:
        obs = np.array([-20, -20])
            
        p0v = numpyArray2DToVector3Array(p0, self.numD)
        v0v = numpyArray2DToVector3Array(v0, self.numD)
        a0v = numpyArray2DToVector3Array(a0, self.numD)
        p1v = numpyArray2DToVector3Array(p1, self.numD)
        v1v = numpyArray2DToVector3Array(v1, self.numD)
        a1v = numpyArray2DToVector3Array(a1, self.numD)
        obsv = numpyArray2DToVector3Array(obs, self.numD)
    
        print p0
        print p1
    
        resp = self.genPath(self.N, self.numD, T, h, R, p0v, v0v, a0v, p1v, v1v, a1v, obsv)
        
        ma = MarkerArray()
        client_array = []
        goal_array = []
        pa = PathArray()
        for i in range(self.N):
            client, goal, marker = sendPath(self.veh[i], resp.trajectories[i], self.current_goal[i].pos.z, i)
            # broadcast path marker for visualization purposes
            ma.markers.append(marker)
            client_array.append(client)
            goal_array.append(goal)
            
            path = Path()
            path.header.frame_id = self.veh[i]
            for j in range(len(resp.trajectories[i].pos)):
                path.x.append(resp.trajectories[i].pos[j].vector.x)
                path.y.append(resp.trajectories[i].pos[j].vector.y)
            pa.path_array.append(path)
            
                
        
        self.pub_marker.publish(ma)
        self.pub_path_array.publish(pa)
        for i in range(self.N):
            client_array[i].send_goal(goal_array[i])
            
        client_array[self.N-1].wait_for_result()
        return True


if __name__ == '__main__':
    ns = rospy.get_namespace()
    try:
        rospy.init_node('multi_vehicle_path_server')
        c = MultiVehicleWaypointServer(['BQ01s', 'BQ02s', 'BQ03s', 'BQ04s'])
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

