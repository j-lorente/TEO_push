# -*- coding: utf-8 -*-
"""

@author: Juan Lorente
MASTERS' THESIS

This is for plotting data from the main program

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

#Get initial time
t0 = time.time()

while True:
     
        #Read the data from the YARP port
        output = yarp.Bottle()
        input_port.read(output)
        ZMP=output.get(0).asDouble()
        #setpoint=output.get(1).asDouble()
        
        #Get actual time
        t = time.time() - t0
        
        #Plot graph
        #plt.plot([t], [ZMP],'bo', [t], [setpoint],'ro')
        plt.plot([t], [ZMP],'ro')
        plt.axis([0, 50, -5, 5])
        plt.xlabel('Time (s)')
        plt.ylabel('ZMP (cm)')
        plt.show()
        
        plt.pause(0.1)
        

