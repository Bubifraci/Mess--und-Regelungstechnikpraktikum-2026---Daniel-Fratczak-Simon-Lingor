import matplotlib.pyplot as plt
import numpy as np
import yaml

x = []
y = []
writeData = []

with open('path.dat', 'r') as file:
    for line in file:
        valueset = line.split()
        if(len(valueset)>1):
            x.append(float(valueset[0]))
            y.append(float(valueset[1]))
        else:
            writeData.append(line.strip())

x_origin = x[0]
y_origin = y[0]

x_transformed = []
y_transformed = []

for i in range(len(x)):
    x_transformed.append(x[i]-x_origin)
    y_transformed.append(y[i]-y_origin)

scale = 0.5

x_scaled = []
y_scaled = []

for i in range(len(x)):
    x_scaled.append(x_transformed[i]*scale)
    y_scaled.append(y_transformed[i]*scale)
    writeData.append(f"{x_scaled[i]} {y_scaled[i]}")

with open("modified.dat", "w", encoding="utf-8") as f:
    for line in writeData:
        f.write(line + "\n")

fig, ax = plt.subplots()
ax.plot(x, y, color='red', linewidth=1.5, label='Old path')
ax.plot(x_scaled, y_scaled, color='green', linewidth=1.5, label='Modified path')
ax.axis('equal')

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Path")
ax.legend()
plt.show()