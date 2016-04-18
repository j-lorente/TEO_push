# -*- coding: utf-8 -*-
"""

@author: Juan Lorente
MASTER'S THESIS

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
strategy = data[:,5]

final_x = t[len(data)-1]

plt.subplot(311)
plt.plot(t, acc, linestyle='-', color='r', label='Sensor acceleration')
plt.plot(t, acc_avg, linestyle='-', color='b', label='Average acceleration')
plt.axis([0, final_x, -11, 11])
plt.title('Acceleration in X')
plt.xlabel('s')
plt.ylabel('m/s^2')
plt.grid(True)
plt.legend()

plt.subplot(312)
plt.plot(t, setpoint, linestyle='-', color='r', label='Setpoint')
plt.plot(t, zmp, linestyle='-', color='b', label='Current ZMP')
plt.axis([0, final_x, -11, 11])
plt.title('Zero Moment Point')
plt.xlabel('s')
plt.ylabel('cm')
plt.grid(True)
plt.legend()

plt.subplot(313)
plt.plot(t, strategy, linestyle='-', color='g', label='0 = Ankle / 5 = Hip')
plt.axis([0, final_x, -11, 11])
plt.title('Strategy')
plt.xlabel('s')
plt.grid(True)
plt.legend()

plt.show()

