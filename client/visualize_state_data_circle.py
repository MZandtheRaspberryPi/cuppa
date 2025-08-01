import math
import time
import threading


from serial_comms import SerialCommunicator


import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

import numpy as np

def get_x_rotation(theta):

    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

def get_y_rotation(theta):

    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def main():
    comms = SerialCommunicator()
    comms.start()


    angles = np.linspace(0, 2 * np.pi, 36)
    global points
    points = np.zeros((angles.shape[0], 3))

    points[:, 0] = np.cos(angles)
    points[:, 1]  = np.sin(angles)
    points[:, 2]  = np.ones(angles.shape)


    # and plot everything
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    xyz_data, = ax.plot(points[:, 0], points[:, 1], points[:, 2])

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)

    def get_sensor_data(frame):
        global points

        state, cov = comms.get_state_cov_data()

        x_rot = get_x_rotation(state[0])
        y_rot = get_y_rotation(state[1])

        new_points = (y_rot @ x_rot @ points.T).T

        xyz_data.set_xdata(new_points[:, 0])
        xyz_data.set_ydata(new_points[:, 1])
        xyz_data.set_3d_properties(new_points[:, 2])

        return xyz_data


    ani = animation.FuncAnimation(fig, get_sensor_data, interval=10)


    plt.show()

    comms.stop()

    plt.close('all')


if __name__ == "__main__":
    main()
