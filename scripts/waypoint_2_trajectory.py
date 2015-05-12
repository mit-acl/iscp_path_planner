#!/usr/bin/env python
import roslib
roslib.load_manifest('path_planner')
import rospy
import numpy as np
import actionlib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from path_planner.msg import *
from path_planner.srv import GenPath
from raven_rviz.msg import QuadMsg


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


class Waypoint2Trajectory():

    def __init__(self):
        self.quad_msg = QuadMsg
        self.pose = Pose()
        self.twist = Twist()

    def waypointCB(self, data):
        self.quad_msg = data
        print self.quad_msg

    def poseCB(self, data):
        self.pose = data.pose

    def twistCB(self, data):
        self.twist = data.twist

if __name__ == '__main__':
    rospy.init_node('send_path')

    rospy.wait_for_service('/gen_path')
    genPath = rospy.ServiceProxy('/gen_path', GenPath)
    print "got path service"

    c = Waypoint2Trajectory()
    rospy.Subscriber("/quad_goal", QuadMsg, c.waypointCB)
    rospy.Subscriber("pose", PoseStamped, c.poseCB)
    rospy.Subscriber("vel", TwistStamped, c.twistCB)
    rospy.spin()

    p0 = np.array([[0, 0], [2, 0], [1, 1], [1, -1]])
    p1 = np.array([[2, 0], [0, 0], [1,-1], [1, 1]])
    v0 = np.zeros((4,2))
    v1 = np.zeros((4,2))
    a0 = np.zeros((4,2))
    a1 = np.zeros((4,2))

    p0 = np.array([[0, 0], [2, 0], [1, -1]])
    p1 = np.array([[2, 0], [0, 0], [1,1]])
    v0 = np.zeros((3,2))
    v1 = np.zeros((3,2))
    a0 = np.zeros((3,2))
    a1 = np.zeros((3,2))

    p0 = np.array([[0, 0], [0, -2]])
    p1 = np.array([[0, -2], [0, 0]])
    v0 = np.zeros((2, 2))
    v1 = np.zeros((2, 2))
    a0 = np.zeros((2, 2))
    a1 = np.zeros((2, 2))

    p0 = np.array([0, 0])
    p1 = np.array([0, -3])
    v0 = np.zeros((1, 2))
    v1 = np.zeros((1, 2))
    a0 = np.zeros((1, 2))
    a1 = np.zeros((1, 2))

    N = 1# p0.shape[0]
    numD = 2# p0.shape[1]
    h = 0.02
    T = 3.0
    R = 0.8

    p0v = numpyArray2DToVector3Array(p0, numD)
    v0v = numpyArray2DToVector3Array(v0, numD)
    a0v = numpyArray2DToVector3Array(a0, numD)
    p1v = numpyArray2DToVector3Array(p1, numD)
    v1v = numpyArray2DToVector3Array(v1, numD)
    a1v = numpyArray2DToVector3Array(a1, numD)

    resp = genPath(N, numD, T, h, R, p0v, v0v, a0v, p1v, v1v, a1v)

    client0, goal0, marker0 = sendPath('BQ02', resp.trajectories[1])
    client1, goal1, marker1 = sendPath('BQ04', resp.trajectories[0])

    # broadcast path marker for visualization purposes
    ma = MarkerArray()
    ma.markers.append(marker0)
    ma.markers.append(marker1)
    pubMarker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True)
    pubMarker.publish(ma)

    ans=raw_input('accept solution?')

    if ans == 'y':

        client0.send_goal(goal0)
        client1.send_goal(goal1)
        client0.wait_for_result()
        client1.wait_for_result()
