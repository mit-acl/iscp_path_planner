#!/usr/bin/env python

from PyQt4 import QtGui
from PyQt4 import QtCore
from ProjTransform import ProjTransform
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class QtWidgetWindow(QtGui.QWidget):
    def __init__(self, size=(5.0, 4.0), dpi=100):
        QtGui.QWidget.__init__(self)
        self.move(0,0)
        #self.showMaximized()
        #self.resize(3840,1200)
        
        
        #self.resize(4800,1920)
        self.resize(1920,1080)
        #self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.fig = Figure(size, dpi=dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        #self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        #self.toolbar = NavigationToolbar(self.canvas, self)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_xlim(xmin=0,xmax =+1)
        self.axes.set_ylim(ymin=0,ymax= +1)

        #self.axes.set_xlim(xmin=-7,xmax =+3)
        #self.axes.set_ylim(ymin=-7,ymax= +3)
        
        self.fig.patch.set_visible(False)
        
        self.vbox = QtGui.QVBoxLayout()
        #self.vbox.addWidget(self.toolbar)
        self.vbox.addWidget(self.canvas)
        
        self.setLayout(self.vbox)
        
        self.axes.clear()
        self.axes.get_xaxis().set_visible(False)
        self.axes.get_yaxis().set_visible(False)
        
        self.fig.tight_layout(pad=-0.125)
        #py.subplots_adjust(bottom =0, left = 0)
        #self.ax = self.fig.add_axes([0,0,1,1])
        #self.ax.spines['top'].set_active(False)
        #self.ax.axis('off')        
        #self.axes.grid(True)
        #self.axes.grid(True)
        self.canvas.draw()

        x1,y1 = -2.180,2.947
        x2,y2 = 1.959, 2.917
        x3,y3 = 2.373,-7.337
        x4,y4 = -2.045,-7.337
        self.viconTransform = ProjTransform(x1,y1,x2,y2,x3,y3,x4,y4)
    
    def getFigure(self):
        return self.fig
        
    def draw(self):
        self.canvas.draw()
