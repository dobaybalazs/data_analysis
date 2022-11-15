#!/usr/bin/env python
from xml.dom.minidom import Element
import rospy 
from std_msgs.msg import Float32MultiArray
import sys, random
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from scipy import signal

class DataHandler():
    def __init__(self):
        rospy.Subscriber("/ambient_data",Float32MultiArray,self.ambientDataCallBack)
        rospy.Subscriber("/intensity_data",Float32MultiArray,self.intensityDataCallBack)
        rospy.Subscriber("/reflectivity_data",Float32MultiArray,self.reflectivityDataCallBack)

        self.app = QtGui.QApplication([]) 
        self.win = pg.GraphicsWindow(title='Values')
        # k = black, w = white
        self.win.setBackground('k')
        self.win.resize(1500,900)
        self.pa = self.win.addPlot(title='Ambient values(raw)')
        self.pi = self.win.addPlot(title='Intensity values(raw)')
        self.pre = self.win.addPlot(title='Reflectivity values(raw)')
        self.win.nextRow()
        self.pa2 = self.win.addPlot(title='Ambient values(smoothed)')
        self.pi2 = self.win.addPlot(title='Intensity values(smoothed)')
        self.pre2 = self.win.addPlot(title='Reflectivity values(smoothed)')
        self.win.nextRow()
        self.pa3 = self.win.addPlot(title='Ambient values(Histogram)')
        self.pi3 = self.win.addPlot(title='Intensity values(Histogram)')
        self.pre3 = self.win.addPlot(title='Reflectivity values(Histogram)')

        # first row
        self.curve_a = None
        self.curve_i = None
        self.curve_re = None
        #second row
        self.curve_a2 = None
        self.curve_i2 = None
        self.curve_re2 = None
        #third row
        self.curve_a3 = None
        self.curve_i3 = None
        self.curve_re3 = None

        self.amb_data = None
        self.int_data = None
        self.refl_data = None

        self.wd_size = 53
        self.f_order = 1

    def ambientDataCallBack(self,msg):
        self.amb_data = msg.data[0:msg.layout.dim[0].size:1]

    def intensityDataCallBack(self,msg):
        self.int_data = msg.data[0:msg.layout.dim[0].size:1]

    def reflectivityDataCallBack(self,msg):
        self.refl_data = msg.data[0:msg.layout.dim[0].size:1]

    def displayData(self):
        rospy.init_node('visualize_histogram',anonymous=True)
        rate = rospy.Rate(20)
        setAValue = True
        setIValue = True
        setREValue = True
        setAValue2 = True
        setIValue2 = True
        setREValue2 = True
        setAValue3 = True
        setIValue3 = True
        setREValue3 = True
        while not rospy.is_shutdown():
            resolution = np.linspace(0,1,100)
            #FIRST ROW
            if(self.amb_data is not None):
                try:
                    x = np.linspace(1,len(self.amb_data),len(self.amb_data))
                    y = self.amb_data
                    if setAValue:
                        self.pa.setYRange(0,1) #setting fix y range
                        self.curve_a = self.pa.plot(x,y,pen=(255,0,0), fillLevel=0, brush=(255, 0, 0, 80))
                        setAValue = False         
                    self.curve_a.setData(x,y)                 
                    self.curve_a.setPos(0,0)                
                    self.app.processEvents()
                except:
                    continue
            if(self.int_data is not None):
                try:
                    x = np.linspace(1,len(self.int_data),len(self.int_data))
                    y = self.int_data  
                    if setIValue:
                        self.pi.setYRange(0,1) #setting fix y range
                        self.curve_i = self.pi.plot(x,y,pen=(0,255,0), fillLevel=0, brush=(0, 255, 0, 80))
                        setIValue = False                  
                    self.curve_i.setData(x,y)                 
                    self.curve_i.setPos(0,0)              
                    self.app.processEvents()
                except:
                    continue
            if(self.refl_data is not None):
                try:
                    x = np.linspace(1,len(self.refl_data),len(self.refl_data))
                    y = self.refl_data 
                    if setREValue:
                        self.pre.setYRange(0,1) #setting fix y range
                        self.curve_re = self.pre.plot(x,y,pen=(255,255,0), fillLevel=0, brush=(255, 255, 0, 80))
                        setREValue = False                  
                    self.curve_re.setData(x,y)                 
                    self.curve_re.setPos(0,0)                 
                    self.app.processEvents()
                except:
                    continue
            #SECOND ROW
            if(self.amb_data is not None):
                try:
                    x = np.linspace(1,len(self.amb_data),len(self.amb_data))
                    y = signal.savgol_filter(self.amb_data,self.wd_size,self.f_order) #(data,window_size,order)
                    if setAValue2:
                        self.pa2.setYRange(0,1) #setting fix y range
                        self.curve_a2 = self.pa2.plot(x,y,pen=(255,0,0), fillLevel=0, brush=(255, 0, 0, 80))
                        setAValue2 = False         
                    self.curve_a2.setData(x,y)                 
                    self.curve_a2.setPos(0,0)                
                    self.app.processEvents()
                except:
                    continue
            if(self.int_data is not None):
                try:
                    x = np.linspace(1,len(self.int_data),len(self.int_data))
                    y = signal.savgol_filter(self.int_data,self.wd_size,self.f_order) #(data,window_size,order)  
                    if setIValue2:
                        self.pi2.setYRange(0,1) #setting fix y range
                        self.curve_i2 = self.pi2.plot(x,y,pen=(0,255,0), fillLevel=0, brush=(0, 255, 0, 80))
                        setIValue2 = False                  
                    self.curve_i2.setData(x,y)                 
                    self.curve_i2.setPos(0,0)              
                    self.app.processEvents()
                except:
                    continue
            if(self.refl_data is not None):
                try:
                    x = np.linspace(1,len(self.refl_data),len(self.refl_data))
                    y = signal.savgol_filter(self.refl_data,self.wd_size,self.f_order) #(data,window_size,order) 
                    if setREValue2:
                        self.pre2.setYRange(0,1) #setting fix y range
                        self.curve_re2 = self.pre2.plot(x,y,pen=(255,255,0), fillLevel=0, brush=(255, 255, 0, 80))
                        setREValue2 = False                  
                    self.curve_re2.setData(x,y)                 
                    self.curve_re2.setPos(0,0)                 
                    self.app.processEvents()
                except:
                    continue
            #THIRD ROW
            if(self.amb_data is not None):
                y,x = np.histogram(self.amb_data,bins=resolution)
                if setAValue3:
                    self.curve_a3 = self.pa3.plot(x,y,pen=(255,0,0),stepMode=True, fillLevel=0, brush=(255, 0, 0, 80))
                    setAValue3 = False         
                self.curve_a3.setData(x,y)                 
                self.curve_a3.setPos(0,0)                
                self.app.processEvents()
            if(self.int_data is not None):
                y,x = np.histogram(self.int_data,bins=resolution)   
                if setIValue3:
                    self.curve_i3 = self.pi3.plot(x,y,pen=(0,255,0),stepMode=True, fillLevel=0, brush=(0, 255, 0, 80))
                    setIValue3 = False                  
                self.curve_i3.setData(x,y)                 
                self.curve_i3.setPos(0,0)              
                self.app.processEvents()
            if(self.refl_data is not None):
                y,x = np.histogram(self.refl_data,bins=resolution)   
                if setREValue3:
                    self.curve_re3 = self.pre3.plot(x,y,pen=(255,255,0),stepMode=True, fillLevel=0, brush=(255, 255, 0, 80))
                    setREValue3 = False                  
                self.curve_re3.setData(x,y)                 
                self.curve_re3.setPos(0,0)                 
                self.app.processEvents()
            rate.sleep()
        self.win.closeEvent()
        
    
            


if __name__ == '__main__':
    dh = DataHandler()
    dh.displayData()

    pg.QtGui.QApplication.exec_()

    