#!/usr/bin/env python
import roslib
roslib.load_manifest('iscp_path_planner')
import rospy
import numpy as np
import actionlib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from acl_msgs.msg import Waypoint
from acl_msgs.msg import QuadGoal as Goal
from acl_msgs.srv import MultiVehWaypoint
from acl_msgs.srv import MultiVehTakeoffLand
from acl_msgs.srv import MultiVehPreCompute
from acl_msgs.srv import GenPath
from acl_msgs.msg import QuadPathArray as PathArray
from acl_msgs.msg import QuadPath as Path
import plot_trajectories

def randTestCaseACL(numAgent,R):
    
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
    
    '''initialize'''
    p0 = np.zeros((numAgent,2))
    p1 = np.zeros((numAgent,2))
    v0 = np.zeros((numAgent,2))
    v1 = np.zeros((numAgent,2))
    a0 = np.zeros((numAgent,2))
    a1 = np.zeros((numAgent,2))
    valid = 1
    
    
    ''' starting points '''
    for i in range(numAgent):
        counter = 0;
        while (counter < 1000):
            p0[i,0] = x_min + (x_max-x_min)*np.random.rand()
            p0[i,1] = y_min + (y_max-y_min)*np.random.rand()
            
            '''not too close to other points'''
            valid_point = 1;
            for j in range(i):
                if np.linalg.norm(p0[j,:]-p0[i,:]) < R:
                    valid_point = 0
                    break
                
            '''corner'''
            if (p0[i,0] < x_cor and p0[i,1] >y_cor):
                valid_point = 0
                
            '''pole'''
            if (p0[i,0] < pole_x_max and p0[i,0] > pole_x_min and p0[i,1] <pole_y_max and p0[i,1] >pole_y_min):
                valid_point = 0
                
            '''valid'''
            if valid_point == 1:
                break
            
            counter = counter + 1
        
        if counter >=100:
            valid = 0
            return p0,p1,v0,v1,a0,a1
        
    ''' end points '''
    for i in range(numAgent):
        counter = 0;
        while (counter < 1000):
            p1[i,0] = x_min + (x_max-x_min)*np.random.rand()
            p1[i,1] = y_min + (y_max-y_min)*np.random.rand()
            
            '''not too close to other points'''
            valid_point = 1;
            for j in range(i):
                if np.linalg.norm(p1[j,:]-p1[i,:]) < R:
                    valid_point = 0
                    break
                
            '''corner'''
            if (p1[i,0] < x_cor and p1[i,1] >y_cor):
                valid_point = 0
                
            '''pole'''
            if (p1[i,0] < pole_x_max and p1[i,0] > pole_x_min and p1[i,1] <pole_y_max and p1[i,1] >pole_y_min):
                valid_point = 0
                
            '''valid'''
            if valid_point == 1:
                break
            
            counter = counter + 1
        
        if counter >= 100:
            valid = 0
            return p0,p1,v0,v1,a0,a1
    print 'starting', p0
    print 'ending', p1
    return p0,p1,v0,v1,a0,a1
        



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

    print m.id

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


def sendPath(name, path):

    pathmarker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    initMarker(name, pathmarker)

    for i in range(len(path.pos)):
        p = Point()
        p.x = path.pos[i].vector.x
        p.y = path.pos[i].vector.y
        p.z = path.pos[i].vector.z
        path.pos[i].vector.z = 0.5
        pathmarker.points.append(p)

    client = actionlib.SimpleActionClient('/' + name + '/broadcast_trajectory',
                                          BroadcastTrajectoryAction)

    client.wait_for_server()
    goal = BroadcastTrajectoryGoal(path, False)

    return client, goal, pathmarker


if __name__ == '__main__':
    rospy.init_node('send_path')

    rospy.wait_for_service('/gen_path')
    genPath = rospy.ServiceProxy('/gen_path', GenPath)
    print "got path service"


    p0 = np.array([[-0.9, -4.3], [-5.5, -4.3], [-3.2,-2], [-3.2,-6.5]])
    p1 = np.array([[-5.5, -4.3], [-0.9, -4.3], [-3.2,-6.5], [-3.2,-2]])
    v0 = np.zeros((4,2))
    v1 = np.zeros((4,2))
    a0 = np.zeros((4,2))
    a1 = np.zeros((4,2))

    '''
    p0 = np.array([[0, 0], [2, 0], [1, -1]])
    p1 = np.array([[2, 0], [0, 0], [1,1]])
    v0 = np.zeros((3,2))
    v1 = np.zeros((3,2))
    a0 = np.zeros((3,2))
    a1 = np.zeros((3,2))



    
    p0 = np.array([[-1,-5], [-2, -5]])
    p1 = np.array([[-1, -2], [-2, -5]])
    v0 = np.zeros((2, 2))
    v1 = np.zeros((2, 2))
    a0 = np.zeros((2, 2))
    a1 = np.zeros((2, 2))
    
    
    p0 = np.array([0.05465195,  0.62567344])
    p1 = np.array([-4.17477261, -5.60593838])
    v0 = np.zeros((1, 2))
    v1 = np.zeros((1, 2))
    a0 = np.zeros((1, 2))
    a1 = np.zeros((1, 2))
    '''

    

    N = 4# p0.shape[0]
    numD = 2# p0.shape[1]
    h = 0.05
    T = 1
    R = 0.8
    p0,p1,v0,v1,a0,a1 = randTestCaseACL(N,R)
    
    p0v = numpyArray2DToVector3Array(p0, numD)
    v0v = numpyArray2DToVector3Array(v0, numD)
    a0v = numpyArray2DToVector3Array(a0, numD)
    p1v = numpyArray2DToVector3Array(p1, numD)
    v1v = numpyArray2DToVector3Array(v1, numD)
    a1v = numpyArray2DToVector3Array(a1, numD)

    resp = genPath(N, numD, T, h, R, p0v, v0v, a0v, p1v, v1v, a1v,[])

    #print resp.trajectories

    plot_trajectories.plot(resp.trajectories, False)

    # client0, goal0, marker0 = sendPath('BQ02', resp.trajectories[0])
    # #client1, goal1, marker1 = sendPath('BQ04', resp.trajectories[1])

    # # broadcast path marker for visualization purposes
    # ma = MarkerArray()
    # ma.markers.append(marker0)
    # #ma.markers.append(marker1)
    # pubMarker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True, queue_size=1)
    # pubMarker.publish(ma)

    # ans=raw_input('accept solution?')

    # if ans == 'y':

    #     client0.send_goal(goal0)
    #     #client1.send_goal(goal1)
    #     client0.wait_for_result()
    #     #client1.wait_for_result()
