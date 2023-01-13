# importing libraries
import numpy as np
import time
import socket
import struct
from matplotlib import pyplot as plt

DEG_TO_RAD = np.pi/180

class Finger:
    def __init__(self, name=None, position=0):
        self.name = name
        if name != 'thumb':
            self.first_phalangie = np.array([position, 0, 0])
            self.end_original = np.array([position, 0, 1])
            self.end = np.array([position, 0, 1])
        else:
            self.first_phalangie = np.array([0, position, 0])
            self.end_original = np.array([0, position, 0.5])
            self.end = np.array([0, position, 0.5])

    def bend(self, theta):
        if self.name != 'thumb':
            #Rx
            R = np.array([[1,             0,              0],
                          [0, np.cos(theta), -np.sin(theta)],
                          [0, np.sin(theta),  np.cos(theta)]])
            self.end = np.dot(R, self.end_original)
        else:
            #Ry
            R = np.array([[ np.cos(theta), 0, np.sin(theta)],
                          [            0,  1,             0],
                          [-np.sin(theta), 0, np.cos(theta)]])
            self.end = np.dot(R, self.end_original)

class Palm:
    def __init__(self, center):
        self.center = center
        self.fingers = []

    def add_finger(self, Finger):
        self.fingers.append(Finger)

    def plot(self):
        x, y, z = [self.center[0]], [self.center[1]], [self.center[2]]
        for finger in self.fingers:
            x.append(finger.first_phalangie[0])
            x.append(finger.end[0])
            y.append(finger.first_phalangie[1])
            y.append(finger.end[1])
            z.append(finger.first_phalangie[2])
            z.append(finger.end[2])
            for i in range(0, len(x), 1):
                plt.plot(x[i:i+2], y[i:i+2], z[i:i+2], 'ro-')
            x, y, z = [self.center[0]], [self.center[1]], [self.center[2]]

palm = Palm(center=[2.5, 0, -1])
thumb = Finger(name='thumb', position=-1)
index_finger = Finger(name='index', position=1.5)
middle_finger = Finger(name='middle', position=2.5)
ring_finger = Finger(name='index', position=3.5)
pinky_finger = Finger(name='pinky', position=4.5)

fingers = [thumb, index_finger, middle_finger, ring_finger, pinky_finger]
for finger in fingers:
    palm.add_finger(finger)

# Create GUI event loop
plt.ion()
figure = plt.figure()
ax = plt.axes(projection='3d')
plt.xlim(-2, 7)
plt.ylim(-2, 7)
ax.set_zlim(-1,1)

# Hide grid lines
ax.grid(False)

# Hide axes ticks
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])

#Connect to LabView Server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 8089)
client.connect(server_address)

# Loop
toggle = 1
while(True):
    palm.plot()

    for i in range(5):
        str_data = client.recv(9)
        angle = float(str_data.decode('ascii'))
        fingers[i].bend(angle * DEG_TO_RAD)
    # drawing updated values
    figure.canvas.draw()

    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()
    ax.clear()
    plt.xlim(-2, 7)
    plt.ylim(-2, 7)
    ax.set_zlim(-1, 1)
    # Hide grid lines
    ax.grid(False)

    # Hide axes ticks
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    #time.sleep(0.05)

client.sendall(b'Enough data :) Thanks')  # Sending anything back closes the connection
