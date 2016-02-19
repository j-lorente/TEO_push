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
zmp = data[:,1]
setpoint = data[:,2]

plt.plot(t, zmp, linestyle='-', color='b', label='ZMP')
plt.plot(t, setpoint, linestyle='-', color='r', label='setpoint')
plt.axis([0, 10, -11, 11])
plt.xlabel('s')
plt.ylabel('cm')
plt.legend()
plt.show()