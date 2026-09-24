import matplotlib.pyplot as plt
import numpy as np
import yaml

isGio = True
iteration = []
x = []
y = []
theta = []
u = []
omega = []
leftspeed = []
rightspeed = []
#i, x, y, theta, u, omega, leftspeed, rightspeed

with open('pos.dat', 'r') as file:
    for line in file:
        valueset = line.split()
        gioOffset = 0
        if(isGio):
            gioOffset = 1
        else:
            iteration.append(float(valueset[0]))
        x.append(float(valueset[1-gioOffset]))
        y.append(float(valueset[2-gioOffset]))
        print("X: " + valueset[1-gioOffset] + " Y: " + valueset[2-gioOffset])
        theta.append(float(valueset[3-gioOffset]))
        u.append(valueset[4-gioOffset])
        omega.append(float(valueset[5-gioOffset]))
        leftspeed.append(float(valueset[6-gioOffset]))
        rightspeed.append(float(valueset[7-gioOffset]))


fig, ax = plt.subplots()
ax.plot(x, y, color='red', linewidth=1.5, label='Route')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Simulated robot path")
ax.legend()
plt.show()
