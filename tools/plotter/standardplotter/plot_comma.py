import matplotlib.pyplot as plt
import numpy as np
import yaml

x = []
y = []
#i, x, y, theta, u, omega, leftspeed, rightspeed

with open('pos.dat', 'r') as file:
    for line in file:
        valueset = line.split()
        x.append(float(valueset[0]))
        y.append(float(valueset[1]))

fig, ax = plt.subplots()
ax.plot(x, y, color='red', linewidth=1.5)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Intended Route")
plt.show()
