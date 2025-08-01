import math
import time
import threading


from serial_comms import SerialCommunicator


import matplotlib.pyplot as plt
import matplotlib.animation as animation


def main():
    comms = SerialCommunicator()
    comms.start()

    max_data_points = 100

    fig, ax = plt.subplots(figsize=(12,14))

    rot_x = []
    rot_y = []
    rot_z = []

    all_data = [rot_x, rot_y, rot_z]

    rot_x_line, = ax.plot([], [], label="rotation_x_rads", color="red")
    rot_y_line, = ax.plot([], [], label="rotation_y_rads", color="green")
    rot_z_line, = ax.plot([], [], label="rotation_z_rads", color="blue")

    ax.set_ylim(-math.pi, math.pi)
    ax.set_xlim(0, max_data_points)
    ax.set_title("State Estimate")
    ax.set_xlabel("Time")
    ax.legend()

    def get_sensor_data(frame):

        if len(rot_x) >= max_data_points:
            [arr.pop(0) for arr in all_data]
        state, cov = comms.get_state_cov_data()

        rot_x.append(state[0])
        rot_y.append(state[1])
        rot_z.append(state[2])

        rot_x_line.set_xdata(range(len(rot_x)))
        rot_x_line.set_ydata(rot_x)
        rot_y_line.set_xdata(range(len(rot_y)))
        rot_y_line.set_ydata(rot_y)
        rot_z_line.set_xdata(range(len(rot_z)))
        rot_z_line.set_ydata(rot_z)

        return rot_x_line, rot_y_line, rot_z_line


    ani = animation.FuncAnimation(fig, get_sensor_data, interval=10)


    plt.show()

    comms.stop()

    plt.close('all')


if __name__ == "__main__":
    main()
