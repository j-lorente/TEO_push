# -*- coding: utf-8 -*-
"""

@author: Juan Lorente
MASTERS' THESIS

This is for plotting data from the main program

"""

import matplotlib.pyplot as plt
import numpy as np

data=np.loadtxt('build/data.txt')

t = data[:,0]
acc = data[:,1]
acc_avg = data[:,2]
strategy = data[:,3]
#pid_ankle = data[:,2]
#pid_hip = data[:,3]
#setpoint = data[:,4]
#zmp = data[:,5]

initial_x = t[0]
final_x = t[len(data)-1]

plt.subplot(211)
plt.plot(t, acc, linestyle='-', color='r', label='Sensor acceleration')
plt.plot(t, acc_avg, linestyle='-', color='b', label='Average acceleration')
plt.axis([initial_x, final_x, -11, 11])
plt.title('Acceleration in X')
plt.xlabel('s')
plt.ylabel('m/s^2')
plt.grid(True)
plt.legend()

plt.subplot(212)
plt.plot(t, strategy, linestyle='-', color='b', label='0 = Ankle / 1 = Hip')
plt.axis([initial_x, final_x, -11, 11])
plt.title('Strategy')
plt.xlabel('s')
plt.grid(True)
plt.legend()


#plt.subplot(211)
#plt.plot(t, acc, linestyle='-', color='r', label='Acceleration in X [m/s²]')
#plt.plot(t, pid_ankle, linestyle='-', color='b', label='Ankle PID output [deg/s]')
#plt.plot(t, pid_hip, linestyle='-', color='g', label='Hip PID output [deg/s]')
#plt.axis([initial_x, final_x, -11, 11])
#plt.title('PID output')
#plt.xlabel('s')
#plt.grid(True)
#plt.legend()
#
#plt.subplot(212)
#plt.plot(t, zmp, linestyle='-', color='b', label='Current ZMP')
#plt.plot(t, setpoint, linestyle='-', color='r', label='setpoint')
#plt.axis([initial_x, final_x, -11, 11])
#plt.title('Zero Moment Point')
#plt.xlabel('s')
#plt.ylabel('cm')
#plt.grid(True)
#plt.legend()

plt.show()
