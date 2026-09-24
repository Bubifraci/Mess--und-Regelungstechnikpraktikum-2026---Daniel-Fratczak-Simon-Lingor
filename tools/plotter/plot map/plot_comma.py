import matplotlib.pyplot as plt
import numpy as np
import yaml

x_amcl = []
y_amcl = []
yaw_amcl = []
timestamp_amcl = []

x_odom = []
y_odom = []

rotationOdom = 180

#Phi in deg
def rotateVector(x, y, phi):
    phi_rad = np.deg2rad(phi)
    
    rotationMatrix = np.array([
        [np.cos(phi_rad), -np.sin(phi_rad)],
        [np.sin(phi_rad),  np.cos(phi_rad)]
    ])
    
    vector = np.array([x, y])
    return rotationMatrix @ vector

with open('pathDataAMCL.txt', 'r') as file:
    for line in file:
        valueset = line.split()
        x_amcl.append(float(valueset[0]))
        y_amcl.append(float(valueset[1]))
        yaw_amcl.append(float(valueset[2]))
        timestamp_amcl.append(float(valueset[3]))

with open('pathDataOdom.txt', 'r') as file:
    for line in file:
        valueset = line.split()
        x_odom_raw = float(valueset[0])
        y_odom_raw = float(valueset[1])
        rotatedVector = rotateVector(x_odom_raw, y_odom_raw, rotationOdom)
        x_odom.append(rotatedVector[0])
        y_odom.append(rotatedVector[1])

with open('map.yaml', 'r') as yml:
    data = yaml.full_load(yml)

resolution = data.get('resolution')
originRaw = data.get('origin')
origin_x, origin_y = originRaw[0], originRaw[1]

img = plt.imread("map.pgm")
height, width = img.shape[:2]

fig, ax = plt.subplots()
ax.imshow(img,
          cmap="gray",
          extent=[origin_x, origin_x + (width * resolution), origin_y, origin_y + (height * resolution)],
          origin="lower")
ax.plot(x_odom, y_odom, color='red', linewidth=1.5, label='Odom based route')
ax.plot(x_amcl, y_amcl, color='green', linewidth=1.5, label='AMCL based route')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Robot path on Map")
ax.legend()
plt.show()
