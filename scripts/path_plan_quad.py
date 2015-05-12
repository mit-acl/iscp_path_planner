#!/usr/bin/env python
import rospy
import numpy as np
from raven_rviz.msg import Waypoint
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from raven_utils import raven_utils as ru
from visualizer_iap.msg import Map
from scipy import spatial

from aclpy import utils

TAKEOFF = 1
DISABLE = 2
RESET_INTEGRATORS = 4
ATTITUDE = 5

class Planner():

    def __init__(self):

        points = np.array([[ 1.8004953,  2.6758396],
[-1.9536396,  -7.05583742],
[ 0.1480746,  2.7116714],
[-0.19195949, -6.625461  ]])

        points = np.array([[ 1.8004953,  2.6758396],
[-0.64157551, -3.50260979]])

#        points = np.array([[ 1.8004953,  2.6758396],
#[-1.9536396,  -7.05583742],
#[-1.08081196,  2.34269874],
#[-0.19195949, -6.625461  ]])

        self.x = points[:,0] #np.array([1.93, -2, -2, 0.57, 1.9, -0.43])
        self.y = points[:,1] #np.array([2.22, -6.2, -4.5, 2.12, 0.5, -6.1])


        self.z = 1.5*np.ones(len(self.x))
        self.pose = Pose()
        self.pub = rospy.Publisher('quad_waypoint', Waypoint, latch=True, queue_size=100)
        self.name = rospy.get_namespace()
        self.name = self.name[1:-1]
        self.quad_init = False
        self.current_yaw = 0
        self.kdtree = None
        self.init_explor = False
        self.upucc_at_goal = False
        self.map = Map()
        self.moving_to_wp = False
        self.wp_x = 0.0
        self.wp_y = 0.0
        self.grey_cells = None
        
    '''
    def greyWaypoints(rrt_waypoints,current_map)
        for i in xrange(1,len(rrt_waypoints))
            #waypointCenter[i] = rrt waypoint position moved to grid center
            
            cellColor = current_map.cells[index].color
            if cellColor.r == 0.2 and cellColor.b == 0.2 and cellColor.g == 0.2 and cellColor.a == 1: 
                waypoint_unknown[i] = 1
                setToVisit.add(waypointCenter[i])
            else:
                waypoint_unknown[i] = 0
        if len(setToVisit)==1:
            toVisit = setToVisit #first and only element
        if len(setToVisit)==0:
            #toVisit = goal #go to goal?
        else:
            toVisit = setToVisit.pop([0]) #1st element
            toVisit.add(setToVisit.pop[0]) #2nd element
        waypointsToVisit = grid_centers[toVisit]
        return waypointsToVisit
    '''


    def build_kdtree(self, map_data):
        if self.kdtree == None:
        # build kdtree of the cells
            cells = []
            for i in range(map_data.num_x_cells*map_data.num_y_cells):
                #raven_pos = np.array([map_data.cells[i].center.x,
                #                      map_data.cells[i].center.y])
                #vicon_pos = self.vicon_transform.raven2vicon(raven_pos)
                cells.append([map_data.cells[i].center.x,
                              map_data.cells[i].center.y])
            self.kdtree = spatial.KDTree(cells)


    def sendPaths(self, event):
        vel = 1.0
    
        if not self.quad_init:
            print 'Waiting for vehicle data'
            return

        # send takeoff command
        if not self.init_explor:
            self.init_explor = True
            for i in range(2):
                self.send_waypoint(self.pose.position.x, self.pose.position.y, 0.1, 1.0,
                                   a=1.0, takeoff=True, yaw=self.current_yaw)
                rospy.sleep(0.1)

            self.send_waypoint(self.x[0], self.y[0], self.z[0],
                               0.3, a=0.5, yaw=self.current_yaw)

            # send waypoints
            for i in range(1,len(self.x)):
                print self.x[i]
                self.send_waypoint(self.x[i], self.y[i], self.z[i], vel)
                #rospy.sleep(3)

            self.wp_x = self.x[-1]
            self.wp_y = self.y[-1]
            self.moving_to_wp = True

            # head to ground
            #self.send_waypoint(self.x[-1], self.y[-1], 0.01, 0.2, a=0.2)
            #rospy.sleep(2.3)

            #land
            #self.send_waypoint(self.x[-1], self.y[-1], 0.01, 0.2, a=0.2, motors_off=True)
            #rospy.sleep(1)
            #rospy.signal_shutdown('Done')
            
        # get the first element from the grey list, go there
        if not self.moving_to_wp and self.grey_cells is not None and len(self.grey_cells)>0:
            ind_first = 1
            got_waypoint = False
            while not got_waypoint:
                wp_ind = self.grey_cells[ind_first]
                self.wp_x = self.map.cells[wp_ind].center.x
                self.wp_y = self.map.cells[wp_ind].center.y
                x = self.pose.position.x
                y = self.pose.position.y
                dist = np.sqrt((x-self.wp_x)**2 + (y-self.wp_y)**2)
                if dist < 0.5:
                    ind_first += 1
                else:
                    got_waypoint = True
            print self.wp_x,self.wp_y
            self.send_waypoint(self.wp_x, self.wp_y, self.z[0], vel)
            self.moving_to_wp = True
            
        if self.upucc_at_goal:
            # head to ground
            self.send_waypoint(0.0, 0.0, 0.01, 0.2, a=0.2)
            rospy.sleep(2.3)

            #land
            self.send_waypoint(0.0, 0.0, 0.01, 0.2, a=0.2, motors_off=True)
            rospy.sleep(1)
            rospy.signal_shutdown('Done')
            

    def poseCB(self, data):
        self.quad_init = True
        self.pose = data.pose
        if self.moving_to_wp:
            x = self.pose.position.x
            y = self.pose.position.y
            dist = np.sqrt((x-self.wp_x)**2 + (y-self.wp_y)**2)
            if dist < 0.7:
                self.moving_to_wp = False
        
    def upucc_pose_cb(self, data):
        if data.pose.position.x < -1.6 and data.pose.position.y < -6.8:
            self.upucc_at_goal = True
            print 'upucc at goal!'

    def upucc_path_cb(self, data):
        
        print 'got path'
        
        if self.kdtree is not None:
            self.upucc_path = ru.multiArray2NumpyArray(data)
            x = self.upucc_path[:,0]
            y = self.upucc_path[:,1]
            cell_indices = []
            current_index = -1
            old_index = -2
            for i in range(len(x)):
                cell_dist, current_index = self.kdtree.query(np.array([x[i], y[i]]))
                # check if it is grey
                if self.map.cells[current_index].type == 5: # this is grey
                    if current_index != old_index:
                        cell_indices.append(current_index)
                        old_index = current_index
                        
            print cell_indices
            self.grey_cells = cell_indices
    
    def map_cb(self, data):
        self.map = data
        self.build_kdtree(data)
        

    def send_waypoint(self, x, y, z, v, a=2.5, takeoff=False, motors_off=False, yaw=0.0):
        print 'sending waypoint'
        w = Waypoint()
        w.header.frame_id = self.name
        w.goal_pose.position.x = x
        w.goal_pose.position.y = y
        w.goal_pose.position.z = z
        w.goal_pose.orientation.w = np.math.cos(yaw/2.0)
        w.goal_pose.orientation.x = 0.0
        w.goal_pose.orientation.y = 0.0
        w.goal_pose.orientation.z = np.math.sin(yaw/2.0)
        w.takeoff = takeoff
        w.land = motors_off
        w.acceleration = a
        w.velocity = v

        self.pub.publish(w)
        rospy.sleep(.5)


if __name__ == '__main__':
    ns = rospy.get_namespace()
    try:
        rospy.init_node('path_plan_quad')

        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=BQ01 $ rosrun path_planner broadcast_trajectory.py")
        else:
            c = Planner()
            rospy.Subscriber('pose', PoseStamped, c.poseCB)
            rospy.Subscriber('/uP01s/path', Float64MultiArray, c.upucc_path_cb) #
            rospy.Subscriber('/uP01s/pose', PoseStamped, c.upucc_pose_cb)
           
            rospy.Subscriber('/current_map', Map, c.map_cb)
            rospy.Timer(rospy.Duration(0.1), c.sendPaths)
            rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
