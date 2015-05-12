#!/usr/bin/env python
'''
read trajectory from trajectory.txt
and then generate a trajectory message
'''
import rospy
import roslib
import actionlib
from path_planner.msg import *
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


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

# setup goal marker for visualization in rviz
def initMarker(m):
    m.header.frame_id = "/world"
    m.header.stamp = rospy.Time.now()
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.0

    m.ns = "RAVEN_path"
    m.id = 0
    for letter in str(rospy.get_namespace()):
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

    if rospy.get_namespace() == '/BQ01/':
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
    if rospy.get_namespace() == '/BQ02/':
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
    if rospy.get_namespace() == '/BQ03/':
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
    if rospy.get_namespace() == '/BQ04/':
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0

    m.lifetime = rospy.Duration(10)
    return m


def createTrajMsg():
    data = readFromFile('trajectory.txt')
    traj = Trajectory()
    z_default = 0.5
    z_velDefault = 0
    z_accDefault = 0

    for data_line in data:
        #timestamp
        #time at data_line[0]
        #traj.header.stamp.sec = data_line[0]

        posLine = Vector3Stamped()
        posLine.header.stamp = rospy.Time(data_line[0])
        posLine.vector.x = data_line[1]
        posLine.vector.y = data_line[2]
        posLine.vector.z = z_default
        traj.pos.append(posLine)

        velLine = Vector3Stamped()
        velLine.header = posLine.header
        velLine.vector.x = data_line[3]
        velLine.vector.y = data_line[4]
        velLine.vector.z = z_velDefault
        traj.vel.append(velLine)

        accLine = Vector3Stamped()
        accLine.header = posLine.header
        accLine.vector.x = data_line[5]
        accLine.vector.y = data_line[6]
        accLine.vector.z = z_accDefault
        traj.acc.append(accLine)

    print traj
    return traj


def sendTraj(trajectory):

    pathmarker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    initMarker(pathmarker)
    pubMarker = rospy.Publisher('/RAVEN_world', MarkerArray, latch=True)

    for i in range(len(trajectory.pos)):
        p = Point()
        p.x = trajectory.pos[i].vector.x
        p.y = trajectory.pos[i].vector.y
        p.z = trajectory.pos[i].vector.z
        pathmarker.points.append(p)

    # broadcast path marker for visualization purposes
    ma = MarkerArray()
    ma.markers.append(pathmarker)
    pubMarker.publish(ma)

    client = actionlib.SimpleActionClient('broadcast_trajectory',
                                          BroadcastTrajectoryAction)

    client.wait_for_server()
    goal = BroadcastTrajectoryGoal(trajectory, False)
    client.send_goal(goal)
    client.wait_for_result()


''' for debugging purpose'''
if __name__ == '__main__':
    rospy.init_node('get_traj_from_file')
    S = "hello world from trajectory loader"
    print(S)
    sendTraj(createTrajMsg())
