import yarp
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time
import numpy as np

# Set up the plot
fig, ax = plt.subplots()
ax.set_ylim([-20, 20])
ax.set_title('Line of sight vector')

line1, = ax.plot(np.zeros(100), color='red', label='neck_roll')
line2, = ax.plot(np.zeros(100), color='green', label='neck_pitch')
line3, = ax.plot(np.zeros(100), color='blue', label='neck_yaw')
line4, = ax.plot(np.zeros(100), color='orange', label='tilt')

line5, = ax.plot(np.zeros(100), color='red', linestyle='dotted', label='neck_roll (desired)')
line6, = ax.plot(np.zeros(100), color='green', linestyle='dotted', label='neck_pitch (desired)')
line7, = ax.plot(np.zeros(100), color='blue', linestyle='dotted', label='neck_yaw (desired)')
line8, = ax.plot(np.zeros(100), color='orange', linestyle='dotted', label='tilt (desired)')

ax.legend()

lines1 = [line1, line2, line3, line4]
lines2 = [line5, line6, line7, line8]

# Update the plot
def update(data):
    bottle = input_port.read(False)
    if bottle is not None:
        point = bottle.get(0).asList()
        vector = bottle.get(1).asList()

        if point is not None:

            for i, line in enumerate(lines1):
                

                x = np.roll(line.get_xydata()[:, 0], -1)
                y = np.roll(line.get_xydata()[:, 1], -1)

                x[-1] = x[-2] + 1
                y[-1] = point.get(i).asFloat64() * 180 / np.pi

                line.set_ydata(y)

        if vector is not None:
            for i, line in enumerate(lines2):
                

                x = np.roll(line.get_xydata()[:, 0], -1)
                y = np.roll(line.get_xydata()[:, 1], -1)

                x[-1] = x[-2] + 1
                y[-1] = vector.get(i).asFloat64() * 180 / np.pi

                line.set_ydata(y)

    return lines1, lines2

# Animate the plot
yarp.Network.init()
plt.show(block=False)
plt.pause(10)

input_port = yarp.BufferedPortBottle()
input_port.open("/GazeController/Visualizer/debu:i")
while not yarp.Network.connect("/GazeController/debug:o", "/GazeController/Visualizer/debu:i"):
    time.sleep(0.01)
print('Connected')

ani = animation.FuncAnimation(fig, update)
plt.show()