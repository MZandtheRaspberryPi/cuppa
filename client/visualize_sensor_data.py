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

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(12,14))

    acc_x = []
    acc_y = []
    acc_z = []

    gyro_x = []
    gyro_y = []
    gyro_z = []

    all_data = [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]

    acc_x_line, = axes[0].plot([], [], label='acc_x', color='red')
    acc_y_line, = axes[0].plot([], [], label='acc_y', color='green')
    acc_z_line, = axes[0].plot([], [], label='acc_z', color='blue')
    axes[0].set_ylim(-1, 12) 
    axes[0].set_title("Acceleromter Measurement")
    axes[0].set_xlabel("Time")
    axes[0].legend()

    gyro_x_line, = axes[1].plot([], [], label='gyro_x', color='red')
    gyro_y_line, = axes[1].plot([], [], label='gyro_y', color='green')
    gyro_z_line, = axes[1].plot([], [], label='gyro_z', color='blue')
    axes[1].set_ylim(-math.pi, math.pi) 
    axes[1].set_title("Gyroscope Measurement")
    axes[1].set_xlabel("Time")
    axes[1].legend()
    axes[0].set_xlim(0, max_data_points) 

    def get_sensor_data(frame):

        if len(acc_x) >= max_data_points:
            [arr.pop(0) for arr in all_data]
        acc, gyro = comms.get_sensor_data()
        acc_x.append(acc[0])
        acc_y.append(acc[1])
        acc_z.append(acc[2])
        gyro_x.append(gyro[0])
        gyro_y.append(gyro[1])
        gyro_z.append(gyro[2])

        gyro_x_line.set_xdata(range(len(acc_x)))
        gyro_x_line.set_ydata(gyro_x)
        gyro_y_line.set_xdata(range(len(acc_x)))
        gyro_y_line.set_ydata(gyro_y)
        gyro_z_line.set_xdata(range(len(acc_x)))
        gyro_z_line.set_ydata(gyro_z)
        acc_x_line.set_xdata(range(len(acc_x)))
        acc_x_line.set_ydata(acc_x)
        acc_y_line.set_xdata(range(len(acc_x)))
        acc_y_line.set_ydata(acc_y)
        acc_z_line.set_xdata(range(len(acc_x)))
        acc_z_line.set_ydata(acc_z)
        return acc_x_line, acc_y_line, acc_z_line, gyro_x_line, gyro_y_line, gyro_z_line


    ani = animation.FuncAnimation(fig, get_sensor_data, interval=10)


    plt.show()

    comms.stop()

    plt.close('all')


if __name__ == "__main__":
    main()
