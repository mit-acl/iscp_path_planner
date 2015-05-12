#!/usr/bin/env python
'''
Description: Plots trajectories coming from the path planner

Created on July 11, 2014

@author: Mark Cutler
@email: markjcutler@gmail.com
'''

'''
RAVEN Dimensions in Building 41

                          WALL 1

         +---------------------------------------------+
         |               x=3.1                         |
         |                                             |
         |               ^                             |
         |               |                             |
WALL 2   | y=3.73     <--+  ORIGIN             y=-8.05 |
         |                                             |
         |                                             |
         |                                             |
         |      x=-3.0                   x=-3.0        |
         +------------------+          +------+        |  WALL 6
                            |          |      |        |
               WALL 3       |   y=-3.97|      | y=-4.5 |
                            |          |      |        |
                            |          +------+        |
                            |            x=-3.53       |
                            |                          |
                    WALL 4  |                          |
                            | y=-0.58                  |
                            |                          |
                            |                          |
                            |                          |
                            |          x=-9.42         |
                            +--------------------------+

                                      WALL 5

'''
import rospy
import roslib
roslib.load_manifest('path_planner')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib import rcParams
rc('font', **{'family':'sans-serif', 'sans-serif':['Helvetica']})
rc('text', usetex=True)
rcParams['text.latex.preamble']=[r"\usepackage{amsmath}"]
from path_planner.msg import Trajectory
from geometry_msgs.msg import Vector3Stamped


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

    return data


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

    return traj







def plot(trajectories, show_plot=True):

    fig, ax = plt.subplots()

    colors = ['b','g','r','c','m','k']

    # plot the room
    wall1 = np.array([[3.1, 3.1], [3.73, -8.05]])
    wall2 = np.array([[3.1, -3.0], [3.73, 3.73]])
    wall3 = np.array([[-3.0, -3.0], [3.73, -0.58]])
    wall4 = np.array([[-3.0, -9.42], [-0.58, -0.58]])
    wall5 = np.array([[-9.42, -9.42], [-0.58, -8.05]])
    wall6 = np.array([[-9.42, 3.1], [-8.05, -8.05]])
    pole1 = np.array([[-3.0, -3.0], [-3.97, -4.5]])
    pole2 = np.array([[-3.0, -3.53], [-3.97, -3.97]])
    pole3 = np.array([[-3.53, -3.53], [-3.97, -4.5]])
    pole4 = np.array([[-3.0, -3.53], [-4.5, -4.5]])

    ax.plot(wall1[1,:], wall1[0,:], color='gray', linewidth=4.0)
    ax.plot(wall2[1,:], wall2[0,:], color='gray', linewidth=4.0)
    ax.plot(wall3[1,:], wall3[0,:], color='gray', linewidth=4.0)
    ax.plot(wall4[1,:], wall4[0,:], color='gray', linewidth=4.0)
    ax.plot(wall5[1,:], wall5[0,:], color='gray', linewidth=4.0)
    ax.plot(wall6[1,:], wall6[0,:], color='gray', linewidth=4.0)
    ax.plot(pole1[1,:], pole1[0,:], color='gray', linewidth=4.0)
    ax.plot(pole2[1,:], pole2[0,:], color='gray', linewidth=4.0)
    ax.plot(pole3[1,:], pole3[0,:], color='gray', linewidth=4.0)
    ax.plot(pole4[1,:], pole4[0,:], color='gray', linewidth=4.0)


    '''

                          WALL s1

         +---------------------------------------------+
         |               x=1.1                         |
         |                                             |
         |               ^                             |
         |               |                             |
    WALL s2 | y=1.0   <--+  ORIGIN             y=-6.9  |
         |                                             |
         |                                             |
         |                                             |
         |      x=-2.0                   x=-2.6        |
         +------------------+          +------+        |  WALL s6
                            |          |      |        |
               WALL s3      |   y=-3.70|      | y=-5.0 |
                            |          |      |        |
                            |          +------+        |
                            |            x=-3.90       |
                            |                          |
                    WALL s4 |                          |
                            | y=-1.8                   |
                            |                          |
                            |                          |
                            |                          |
                            |          x=-7.5          |
                            +--------------------------+

                                      WALL s5

    '''

    #plot software room bounds
    wall_s1 = np.array([[1.1, 1.1], [1.0, -6.9]])
    wall_s2 = np.array([[1.1, -2.0], [1.0, 1.0]])
    wall_s3 = np.array([[-2.0, -2.0], [1.0, -1.8]])
    wall_s4 = np.array([[-2.0, -7.5], [-1.8, -1.8]])
    wall_s5 = np.array([[-7.5, -7.5], [-1.8, -6.9]])
    wall_s6 = np.array([[-7.5, 1.1], [-6.9, -6.9]])
    pole_s1 = np.array([[-2.6, -2.6], [-3.7, -5.0]])
    pole_s2 = np.array([[-2.6, -3.9], [-3.7, -3.7]])
    pole_s3 = np.array([[-3.9, -3.9], [-3.70, -5.0]])
    pole_s4 = np.array([[-2.6, -3.9], [-5.0, -5.0]])

    ax.plot(wall_s1[1,:], wall_s1[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(wall_s2[1,:], wall_s2[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(wall_s3[1,:], wall_s3[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(wall_s4[1,:], wall_s4[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(wall_s5[1,:], wall_s5[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(wall_s6[1,:], wall_s6[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(pole_s1[1,:], pole_s1[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(pole_s2[1,:], pole_s2[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(pole_s3[1,:], pole_s3[0,:], color='gray', linewidth=2.0, linestyle='-.')
    ax.plot(pole_s4[1,:], pole_s4[0,:], color='gray', linewidth=2.0, linestyle='-.')
  

    #trajectories = []
    #trajectories.append(createTrajMsg())

    # plot the trajectories
    for i in range(len(trajectories)):
        pos = trajectories[i].pos
        for j in range(len(pos)-1):
            x0 = pos[j].vector.x
            x1 = pos[j+1].vector.x
            y0 = pos[j].vector.y
            y1 = pos[j+1].vector.y
            ax.plot([y0, y1], [x0, x1], color=colors[i], linewidth=2.0,marker='o')
            #if (j+1)%5 == 0:
            #    ax.plot([y0], [x0], color='r', linewidth=2.0,marker='o')
    
    for i in range(len(trajectories)):
        pos = trajectories[i].pos
        ax.plot([pos[0].vector.y], [pos[0].vector.x], color=colors[i], linewidth=2.0,marker='s')

    plt.gca().invert_xaxis()
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    #for tick in ax.xaxis.get_major_ticks():
    #    tick.label.set_fontsize(18)
    #for tick in ax.yaxis.get_major_ticks():
    #    tick.label.set_fontsize(18)
 

    #ax.legend(leg, legend, fancybox=True, fontsize=15, loc=loc)

    ax.set_ylabel('X', size=20)
    ax.set_xlabel('Y', size=20)

    fig.canvas.draw()

    if show_plot:
        plt.show()
    else:
        plt.savefig('path.pdf', format='pdf', transparent=True, bbox_inches='tight')



#plot()
