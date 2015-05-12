#!/usr/bin/env python
'''
Description: Compute simple path when you know you
 are flying in a straight line in a convext region
 with no obstacles

Created on Jan 30, 2015

@author: Mark Cutler
@email: markjcutler@gmail.com
'''

import rospy
import numpy as np
import matplotlib.pyplot as plt
from path_planner.srv import *
from geometry_msgs.msg import Vector3Stamped
from path_planner.msg import Trajectory
from aclpy import utils
import math


class SimpleTrajectory():
    def __init__(self):
        t = 0
        px = py = pz = 0
        vx = vy = vz = 0
        ax = ay = az = 0
        jx = jy = jz = 0
        

def posPoly(p0, v0, v1, a_max):
    c = np.zeros(7) # poly coeffs
    c[6] = p0
    c[5] = v0

    b = np.array([v1-v0,0,0])

    n_max = 1000 # max number of times to run bisection method
    t1a = 0.1 # lower limit in seconds
    t1b = 1000 #upper limit in seconds
    
    # bisection search algorithm
    for i in range(n_max):
    
        t1 = (t1a + t1b)/2.0
    
        A = np.array([[6*t1**5, 5*t1**4, 4*t1**3],
                      [30*t1**4, 20*t1**3, 12*t1**2],
                      [120*t1**3, 60*t1**2, 24*t1]])
        x = np.linalg.solve(A,b)
    
        c[0] = x[0]
        c[1] = x[1]
        c[2] = x[2]
        tm = t1/2.0
        a = 30*c[0]*tm**4 + 20*c[1]*tm**3 + 12*c[2]*tm**2 + 6*c[3]*tm + 2*c[4]
        a = np.abs(a) # only care about the abs of acc
    
        diff = (a - a_max)**2
        if diff < 0.001:
            break
    
        if a < a_max:
            t1b = t1
        else:
            t1a = t1
            
    pos = np.poly1d(c)
    print "Search took " + str(i) + " iterations"
    
    return pos, t1

def computePath1D(p0, p_end, vmax, amax):
    # note, just translate by p0 at the end
    d = p_end - p0
    
    # get section 1 (acceleration)
    t0 = 0
    pos1, t1 = posPoly(0, 0, vmax, amax)
    vel1 = np.polyder(pos1)
    acc1 = np.polyder(pos1, m=2)
    jerk1 = np.polyder(pos1, m=3)
    d1 = pos1(t1)
    
    # get section 3 (decceleration)
    pos3, t3 = posPoly(0, vmax, 0, amax)
    vel3 = np.polyder(pos3)
    acc3 = np.polyder(pos3, m=2)
    jerk3 = np.polyder(pos3, m=3)
    d3 = pos3(t3)
    
    # get middle section (constant vel)
    d2 = d - d1 - d3
    t_offset = 0
    d_offset = 0
    if d2 > 0:
        t2 = d2/vmax
    else:
        t2 = 0
        d2 = 0
        # TODO: figure out what to do in this case!
        # find middle position, and time at which pos1 reaches it
        dm = d/2.0
        d_offset = d1 - dm
        real_roots = np.real(np.roots(pos1-dm))
        real_roots = real_roots[np.where(real_roots < t1)]
        real_roots = real_roots[np.where(real_roots > 0)]
        if len(real_roots) > 1:
            print "Warning: there should only be 1 real root"
        real_roots = min(real_roots)
        t_offset = t1-real_roots
    
    t_end = t1 + t2 + t3 - 2*t_offset
    t = np.linspace(0, t_end, num=t_end*100) # 100 Hz output
    pos = np.zeros(len(t))
    vel = np.zeros(len(t))
    acc = np.zeros(len(t))
    jerk = np.zeros(len(t))
    for i in range(len(t)):
        if t[i] < (t1 - t_offset):
            pos[i] = pos1(t[i])
            vel[i] = vel1(t[i])
            acc[i] = acc1(t[i])
            jerk[i] = jerk1(t[i])
        elif t[i] < t1 - t_offset + t2:
            pos[i] = pos[i-1] + (t[i]-t[i-1])*vmax
            vel[i] = vmax
        elif t[i] <= t1 + t2 + t3 - 2*t_offset:
            t_tmp = t[i]-t2-t1+2*t_offset
            pos[i] = d1 + d2 + pos3(t_tmp) - 2*d_offset
            vel[i] = vel3(t_tmp)
            acc[i] = acc3(t_tmp)
            jerk[i] = jerk3(t_tmp)
            
    pos = pos + p0     
    # set end position to desired end position to account for
    # rounding errors
    if np.sqrt((pos[-1] - p_end)**2) < 0.05:
        pos[-1] = p_end
    
    return pos, vel, acc, jerk, t


def computePath3D(p0, p1, vmax, amax):

    # check inputs
    traj = SimpleTrajectory()
    if not (len(p0) == 3 and len(p1) == 3):
        print "Error: position inputs need to be length 3"
        return traj
    if vmax <= 0:
        print "Error: vmax needs to be positive"
        return traj
    if amax <= 0:
        print "Error: amax needs to be positive"
        return traj

    # first, get the 1D path
    d = utils.distanceN(p0, p1)
    pos, vel, acc, jerk, t = computePath1D(0, d, vmax, amax)
    
    # next, project that path onto the x, y, z coordinates
    beta = math.asin((p1[2]-p0[2])/d) # projection onto x-y plane
    theta = math.atan2(p1[0]-p0[0], p1[1]-p0[1]) # projection onto x, y

    # fill output variables
    traj.t = t
    
    traj.pz = pos*np.sin(beta) + p0[2]
    traj.vz = vel*np.sin(beta)
    traj.az = vel*np.sin(beta)
    traj.jz = jerk*np.sin(beta)
    
    traj.px = pos*np.sin(theta)*np.cos(beta) + p0[0]
    traj.py = pos*np.cos(theta)*np.cos(beta) + p0[1]
    traj.vx = vel*np.sin(theta)*np.cos(beta)
    traj.vy = vel*np.cos(theta)*np.cos(beta)
    traj.ax = acc*np.sin(theta)*np.cos(beta)
    traj.ay = acc*np.cos(theta)*np.cos(beta)
    traj.jx = jerk*np.sin(theta)*np.cos(beta)
    traj.jy = jerk*np.cos(theta)*np.cos(beta)
    
    return traj
    
def simpleTrajectory2PPTrajectory(traj):
    t = Trajectory()
    for i in range(len(traj.t)):
        p = Vector3Stamped()
        v = Vector3Stamped()
        a = Vector3Stamped()
        j = Vector3Stamped()

        p.header.stamp = rospy.Time(traj.t[i])
        p.vector.x = traj.px[i]
        p.vector.y = traj.py[i]
        p.vector.z = traj.pz[i]

        v.header.stamp = rospy.Time(traj.t[i])
        v.vector.x = traj.vx[i]
        v.vector.y = traj.vy[i]
        v.vector.z = traj.vz[i]

        a.header.stamp = rospy.Time(traj.t[i])
        a.vector.x = traj.ax[i]
        a.vector.y = traj.ay[i]
        a.vector.z = traj.az[i]

        j.header.stamp = rospy.Time(traj.t[i])
        j.vector.x = traj.jx[i]
        j.vector.y = traj.jy[i]
        j.vector.z = traj.jz[i]
        
        t.pos.append(p)
        t.vel.append(v)
        t.acc.append(a)
        t.jerk.append(j)
        
    return t

def get_simple_path(req):
    rospy.loginfo('Got path request')
    p0 = [req.p0.x, req.p0.y, req.p0.z]
    p1 = [req.p1.x, req.p1.y, req.p1.z]
    traj = computePath3D(p0, p1, req.vmax, req.amax)
    t = simpleTrajectory2PPTrajectory(traj)    
    rospy.loginfo('Returning path request')
    return GenSimplePathResponse(t)
        
def gen_simple_path_server():
    rospy.init_node('simple_path_planner_server')
    s = rospy.Service('/simple_gen_path', GenSimplePath, get_simple_path)
    rospy.spin()


if __name__ == "__main__":
    gen_simple_path_server()


## DEBUG ##        
#p0 = [2.3, -5.2, 1.2]
#p1 = [.3, 1, .2]
#vmax = 0.9
#amax = 2.0

#traj = computePath3D(p0, p1, vmax, amax)
#t = simpleTrajectory2PPTrajectory(traj)

#f, ax = plt.subplots(nrows=4, sharex=True)
#ax[0].plot(traj.t, traj.px)
#ax[0].plot(traj.t, traj.py)
#ax[0].plot(traj.t, traj.pz)

#ax[1].plot(traj.t, traj.vx)
#ax[1].plot(traj.t, traj.vy)
#ax[1].plot(traj.t, traj.vz)

#ax[2].plot(traj.t, traj.ax)
#ax[2].plot(traj.t, traj.ay)
#ax[2].plot(traj.t, traj.az)

#ax[3].plot(traj.t, traj.jx)
#ax[3].plot(traj.t, traj.jy)
#ax[3].plot(traj.t, traj.jz)

#plt.show()

