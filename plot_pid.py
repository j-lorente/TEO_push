# -*- coding: utf-8 -*-
"""
Created on Thu Feb  4 12:04:20 2016

@author: Juan Lorente
MASTERS' THESIS

### This is for plotting the PID ouput of the main program ###
"""

import matplotlib.pyplot as plt
import yarp
import time

#Initialise YARP
yarp.Network.init()

#Create a YARP port and connect it to the output port
input_port = yarp.Port()
input_port.open("/reader")
yarp.Network.connect("/sender", "/reader")

t0 = time.time()

while True:
     
        #Read the data from the YARP port
        output = yarp.Bottle()
        input_port.read(output)
        ZMP=output.get(0).asDouble()
        vel=output.get(1).asDouble()
        
        t = time.time() - t0
        
        #Graph 1
        plt.subplot(211)
        plt.plot([t], [ZMP],'ro')
        plt.axis([0, 50, -20, 20])
        plt.xlabel('Time (s)')
        plt.ylabel('ZMPx (cm)')
        
        #Graph 2
        plt.subplot(212)
        plt.plot([t], [vel],'bo')
        plt.axis([0, 50, -12, 12])
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (deg/s)')
        
        #Plot graphs
        plt.show()
        
        plt.pause(0.1)
        


