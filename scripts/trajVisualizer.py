#!/usr/bin/env python
from QtWidgetWindow import QtWidgetWindow
import roslib
roslib.load_manifest('iscp_path_planner')
import rospy
import sys, random

from PyQt4 import QtCore, QtGui
#from PyQt4.QtCore import *
#from PyQt4.QtGui import *

import numpy
import threading
#import matplotlib
#matplotlib.use("Agg")


import matplotlib.animation as animation
from geometry_msgs.msg import PoseStamped
from acl_msgs.msg import QuadPathArray as PathArray
#import threading
from std_msgs.msg import Int8
from matplotlib.widgets import CheckButtons
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import time
import tf
import math

#def func(label):
#    if label == '2 Hz': l0.set_visible(not l0.get_visible())
#    elif label == '4 Hz': l1.set_visible(not l1.get_visible())
#    elif label == '6 Hz': l2.set_visible(not l2.get_visible())
#    plt.draw()

class poseSubsriber():
    def __init__(self, name, color, parent):
        self.name = name
        self.color = color
        self.parent = parent
        self.poseSub = rospy.Subscriber(name+'/pose', PoseStamped, self.updatePose)
        
        self.pose = PoseStamped()
        self.psi = 3.14 /4.0
        self.timePoseLastUpdated = 0
        
        self.poseLine1 = Line2D([-0.2,0.2],[0,0])
        self.poseLine1.set_color(color)
        self.poseLine1.set_linewidth(5)
        self.poseLine1.set_visible(True)
        
        self.poseLine2 = Line2D([0,0], [-0.2, 0.2])
        self.poseLine2.set_color(color)
        self.poseLine2.set_linewidth(5)
        self.poseLine2.set_visible(True)
        
        self.traj = Line2D([0,0],[0,-5])
        self.traj.set_color(color)
        self.traj.set_linewidth(2)
        self.traj.set_visible(True) 
        self.goal = [0.0,0.0]
        self.distToGoal = 8888
        self.active = False
        
    def updatePose(self, msg):
        self.active = True
        self.timePoseLastUpdated = time.time()
        self.pose = msg
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.psi = euler[2]

        
        
        c = 0.2
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        yaw = self.psi
        self.distToGoal = math.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2)
        #print self.distToGoal
        
        h_line_x = [x + c * math.cos(yaw), x - c * math.cos(yaw)]
        h_line_y = [y + c * math.sin(yaw), y - c * math.sin(yaw)]
        
        v_line_x = [x - c * math.sin(yaw), x + c * math.sin(yaw)]
        v_line_y = [y + c * math.cos(yaw), y - c * math.cos(yaw)]
        
        h_line_x[0], h_line_y[0] = self.parent.viconTransform.pc(h_line_x[0], h_line_y[0])
        h_line_x[1], h_line_y[1] = self.parent.viconTransform.pc(h_line_x[1], h_line_y[1])
        v_line_x[0], v_line_y[0] = self.parent.viconTransform.pc(v_line_x[0], v_line_y[0])
        v_line_x[1], v_line_y[1] = self.parent.viconTransform.pc(v_line_x[1], v_line_y[1])
        
        
        self.poseLine1.set_xdata(h_line_x)
        self.poseLine1.set_ydata(h_line_y) 
        self.poseLine1.set_visible(True)
        
        self.poseLine2.set_xdata(v_line_x)
        self.poseLine2.set_ydata(v_line_y)
        self.poseLine2.set_visible(True)
        if self.parent.plottingLock.acquire(False) == True:
            self.parent.updatePlot()
            self.parent.plottingLock.release()
        #self.parent.updatePlot()

class OnlineHist(QtWidgetWindow,):
    def __init__(self):
        QtWidgetWindow.__init__(self)

        #Initialize the ROS node
        rospy.init_node('visualizer', anonymous=True)
        self.plotUpdateTime  = rospy.get_time()
        self.plottingLock = threading.Lock()
        
        # hard coded for now, should find active vehicles by parsing path msgs
        self.agentList = ['BQ01','BQ02','BQ03','BQ04', 'BQ01s','BQ02s','BQ03s','BQ04s']
        self.agentColor = {'BQ01':'r','BQ02':'g','BQ03':'b','BQ04':'k','BQ01s':'r','BQ02s':'g','BQ03s':'b','BQ04s':'k'}
        self.setup = False
        # time elapsed
        self.timePoseLastUpdated = dict()
        for i in range(len(self.agentList)):
            self.timePoseLastUpdated[self.agentList[i]] = time.time()
        
        # subscribe to pose from Vicon
        self.veh_list = dict()
        for i in range(len(self.agentList)):
            self.veh_list[self.agentList[i]] = poseSubsriber(self.agentList[i], self.agentColor[self.agentList[i]],self)
            self.axes.add_line(self.veh_list[self.agentList[i]].poseLine1)
            self.axes.add_line(self.veh_list[self.agentList[i]].poseLine2)
            self.axes.add_line(self.veh_list[self.agentList[i]].traj)
        
        # subscribe to paths from path planner
        self.paths_x = dict()
        self.paths_y = dict()
        self.timeTrajLastUpdated = time.time()
        self.timeLastReachedGoal = time.time()
        self.ifAllGoalsReached = False
        for i in range(len(self.agentList)):
            self.paths_x[self.agentList[i]] = []
            self.paths_y[self.agentList[i]] = []
        self.pathArraySub = rospy.Subscriber('/projector_paths', PathArray, self.updateTraj)
        self.setup = True
        

        
    def updateTraj(self, msg):
        self.timeTrajLastUpdated = time.time()
        self.ifAllGoalsReached = False
        for i in range(len(msg.path_array)):
            name = msg.path_array[i].header.frame_id
            self.veh_list[name].traj.set_visible(True)
            self.paths_x[name] = list(msg.path_array[i].x)
            self.paths_y[name] = list(msg.path_array[i].y)
            self.veh_list[name].goal = [msg.path_array[i].x[-1],msg.path_array[i].y[-1]]
            for j in range(len(msg.path_array[i].x)):
                x_tmp = msg.path_array[i].x[j]
                y_tmp = msg.path_array[i].y[j]
                x_tf, y_tf = self.viconTransform.pc(x_tmp, y_tmp)
                self.paths_x[name][j] = x_tf
                self.paths_y[name][j] = y_tf
        
        for i in range(len(self.agentList)):
            self.veh_list[self.agentList[i]].traj.set_xdata(self.paths_x[self.agentList[i]])
            self.veh_list[self.agentList[i]].traj.set_ydata(self.paths_y[self.agentList[i]])
    
        if self.plottingLock.acquire(False) == True:
            self.updatePlot()
            self.plottingLock.release()
            
    def testAllGoalsReached(self):
        for i in range(len(self.agentList)):
            if self.veh_list[self.agentList[i]].distToGoal > 0.1 and self.veh_list[self.agentList[i]].active == True:
                #print self.agentList[i], self.veh_list[self.agentList[i]].distToGoal 
                break
            if i == len(self.agentList)-1:
                return True
        else:
            return False
        
    def updatePlot(self):
        #self.debug()
        plotRate = 20.0
        curTime = rospy.get_time()
        
        if curTime-self.plotUpdateTime > (1/plotRate) and self.setup == True:
            traj_time_elapsed = time.time() - self.timeTrajLastUpdated
            if self.ifAllGoalsReached == False:
                if self.testAllGoalsReached() == True:
                    #print 'allGoalsReached'
                    self.ifAllGoalsReached = True
                    self.timeLastReachedGoal = time.time()
                
            for i in range(len(self.agentList)):
                veh = self.veh_list[self.agentList[i]]
                name = veh.name
                time_elapsed = time.time() - veh.timePoseLastUpdated
                if time_elapsed < 2:
                    # traj
                    # disappearing logic to be improved
                    if traj_time_elapsed > 20:
                        veh.traj.set_visible(False)
                    if self.ifAllGoalsReached == True:
                        if time.time() - self.timeLastReachedGoal > 1.5:
                            veh.traj.set_visible(False)
            
                    
                # haven't seen the vehicle for more than two seconds
                else:
                    veh.active = False
                    veh.poseLine1.set_visible(False)
                    veh.poseLine2.set_visible(False)
                    veh.traj.set_visible(False)
            
            # plotting
            self.canvas.draw()
            #self.artistList.append(self.fig)
            #writer.grab_frame()
            self.plotUpdateTime = curTime
        
    def debug(self):
        self.veh_list['BQ01'].pose.pose.position.x = 0.0
        self.veh_list['BQ01'].pose.pose.position.y = 1.0
        self.veh_list['BQ01'].psi = 0.0
        self.paths_x['BQ01']  = [0.0, 0.0, 0.0, 0.0]
        self.paths_y['BQ01']  = [0.0, -1.0, -2.0, -3.0]
        self.veh_list['BQ01'].timePoseLastUpdated = time.time()
        
                
if __name__ == "__main__":
    
    #FFMpegWriter = animation.writers['avconv']
    #metadata = dict(title='Movie Test', artist='Matplotlib',comment='Movie support!')
    #writer = animation.AVConvFileWriter(fps=15, metadata=metadata)
    #import sys
    #if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        #You MUST be on RAVEN network, otherwise the process will freeze
    #    rospy.init_node('listener', anonymous=True)
    #    QtGui.QApplication.instance().exec_()
    
    
    
    
    app = QtGui.QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()
    
    #window.writer = writer
    #FFMpegWriter = manimation.writers['ffmpeg']
    #metadata = dict(title='Movie Test', artist='Matplotlib',
    #comment='Movie support!')
    #writer = FFMpegWriter(fps=15, metadata=metadata)
    
    #with writer.saving(window.fig, "writer_test.mp4",100):
    
