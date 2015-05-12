#!/usr/bin/env python
import sys


class ProjTransform:
    
    def __init__(self,x1,y1,x2,y2,x3,y3,x4,y4):
        
        
        # this converts crazy vicon coordinates to standard coordinates
        # but the origin isn't the lower left hand corner
        
        self.x1r = x1
        self.x2r = x2
        self.x3r = x3
        self.x4r = x4
        
        self.y1r = y1
        self.y2r = y2
        self.y3r = y3
        self.y4r = y4
        
        self.setGoodCoordinates()
        
    def TV(self,xvr,yvr):
        xv = -(yvr - self.y1r)
        yv = xvr - self.x1r
        
        return xv,yv
    
    
    def setGoodCoordinates(self):
        self.x1,self.y1 = self.TV(self.x1r,self.y1r)
        self.x2,self.y2 = self.TV(self.x2r,self.y2r)
        self.x3,self.y3 = self.TV(self.x3r,self.y3r)
        self.x4,self.y4 = self.TV(self.x4r,self.y4r)
    
    def line1(self,xp):
        l1 = (self.y4 - self.y1)*xp
        return l1
    
    
    def line2(self,xp):
        l2 = self.y2 + (self.y3 - self.y2)*xp
        return l2
    
    def pc(self,xv_raw,yv_raw):
        
        # again we are taking in crazy vicon coordinates
        
        # Transform them to normal vicon coordinates
        
        xv, yv = self.TV(xv_raw,yv_raw)     
        

        
        xp = xv/self.x4
        yp = (yv - self.line1(xp))/(self.line2(xp) - self.line1(xp))
        
        return (xp,yp)
    

if __name__ == "__main__":
    x1,y1 = -2.180,2.947
    x2,y2 = 1.959, 2.917
    x3,y3 = 2.373,-7.337
    x4,y4 = -2.045,-7.337
 

    # create a ProjectorTransform object
    pt = ProjTransform(x1,y1,x2,y2,x3,y3,x4,y4)

    
    print 'The point (0,0) in Vicon maps to '
    print pt.pc(-2.09,2.83)
    

    
    
    
    
    
    
    
    
    
    
    