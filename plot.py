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
setpoint = data[:,3]
zmp = data[:,4]

initial_x = t[0]
final_x = t[len(data)-1]

plt.subplot(211)
plt.plot(t, acc, linestyle='-', color='r', label='IMU acceleration')
plt.plot(t, acc_avg, linestyle='-', color='b', label='Acceleration average')
plt.axis([initial_x, final_x, -5, 5])
plt.title('Acceleration in X')
plt.xlabel('s')
plt.ylabel('m/s^2')
plt.grid(True)
plt.legend()

plt.subplot(212)
plt.plot(t, zmp, linestyle='-', color='b', label='Current ZMP')
plt.plot(t, setpoint, linestyle='-', color='r', label='setpoint')
plt.axis([initial_x, final_x, -11, 11])
plt.title('Zero Moment Point')
plt.xlabel('s')
plt.ylabel('cm')
plt.grid(True)
plt.legend()

plt.show()
