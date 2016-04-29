# -*- coding: utf-8 -*-
"""

@author: Juan Lorente
MASTER'S THESIS

This is for plotting data from the main program

"""

import matplotlib.pyplot as plt
import numpy as np

sagittal_data = np.loadtxt('build/sagittal_data.txt')
frontal_data = np.loadtxt('build/frontal_data.txt')

#sagittal_data = np.loadtxt('sagittal_data.txt')
#frontal_data = np.loadtxt('frontal_data.txt')

t_x = sagittal_data[:,0]
t_y = frontal_data[:,0]
zmp_x = sagittal_data[:,1]
zmp_y = frontal_data[:,1]
setpoint_x = sagittal_data[:,2]
setpoint_y = frontal_data[:,2]
strategy = sagittal_data[:,3]

final_sagittal = t_x[len(sagittal_data)-1]
final_frontal = t_y[len(frontal_data)-1]

plt.subplot(311)
plt.plot(t_x, setpoint_x, linestyle='-', color='r', label='Setpoint')
plt.plot(t_x, zmp_x, linestyle='-', color='b', label='Current ZMP')
plt.axis([0, 138, -25, 25])
plt.title('Sagittal Plane')
plt.xlabel('s')
plt.ylabel('cm')
plt.grid(True)
plt.legend()

plt.subplot(312)
plt.plot(t_y, setpoint_y, linestyle='-', color='r', label='Setpoint')
plt.plot(t_y, zmp_y, linestyle='-', color='b', label='Current ZMP')
plt.axis([0, 138, -11, 11])
plt.title('Frontal Plane')
plt.xlabel('s')
plt.ylabel('cm')
plt.grid(True)
plt.legend()

plt.subplot(313)
plt.plot(t_x, strategy, linestyle='-', color='g', label='0 = Ankle / 5 = Hip')
plt.axis([0, 138, -11, 11])
plt.title('Strategy')
plt.xlabel('s')
plt.grid(True)
plt.legend()

plt.show()

