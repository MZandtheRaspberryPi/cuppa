import math
import time
import threading
from typing import Any


from serial_comms import SerialCommunicator


import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

import numpy as np

def sqrt_np(sigma: Any):
    # Computing diagonalization
    evalues, evectors = np.linalg.eigh(sigma)
    # Ensuring square root matrix exists
    assert (evalues >= 0).all()
    sqrt_matrix = evectors * np.sqrt(evalues) @ evectors.T
    return sqrt_matrix

def get_gaussian_error_ellipses(expected_value: Any, sigma: Any,
                                p_value: float = 0.95,
                                step_value: float = 0.01) -> Any:
    """function to generate plots of gaussian error ellipses

    Args:
        expected_value (Any): numpy nx1 array with means of each gaussian.
        sigma (Any): numpy nxn array covariance matrix
        p_value (float): p value for the error ellipses
        step_value (float, optional): when tracing the ellipse, we step over theta 
            values and calculate the ellipse value at that theta.
            how granular should we step? Defaults to 0.01.

    Returns:
        Any: an Nx2 array, where arr[:, 0] is the x and arr[:, 1] is the y.
            N is determined by step size
            and will be equivalent to 2pi/step_size.
    """
    # comes from solving gaussian pdf integral for the quadratic corresponding to ellipse
    r = math.sqrt(-2 * math.log((1-p_value)))

    ellipses_x = np.arange(0, 2 * math.pi, step_value)
    ellipses_y = np.arange(0, 2 * math.pi, step_value)
    ellipses_x = r * np.cos(ellipses_x)
    ellipses_y = r * np.sin(ellipses_y)

    ellipse_normal = np.zeros((len(ellipses_x), 2))
    ellipse_normal[:, 0] = ellipses_x
    ellipse_normal[:, 1] = ellipses_y
    ellipse_transformed = np.matmul(
        ellipse_normal, sqrt_np(sigma)) + expected_value

    return ellipse_transformed


def main():
    comms = SerialCommunicator()
    comms.start()

    fig, ax = plt.subplots(figsize=(10, 12))

    ellipse_line, = ax.plot([], [])

    mean, = ax.plot([], [], marker="+")

    ax.set_xlabel("x_rotation_radians")
    ax.set_ylabel("y_rotation_radians")

    ax.set_xlim(-2*np.pi, 2*np.pi)
    ax.set_ylim(-2*np.pi, 2*np.pi)

    def get_sensor_data(frame):

        state, cov = comms.get_state_cov_data()
        cov = np.array(cov).reshape((3,3))
        state = np.array(state)

        ellipse = get_gaussian_error_ellipses(state[:2], cov[:2, :2])

        ellipse_line.set_xdata(ellipse[:, 0])
        ellipse_line.set_ydata(ellipse[:, 1])

        mean.set_xdata([state[0]])
        mean.set_ydata([state[1]])

        return ellipse_line, mean


    ani = animation.FuncAnimation(fig, get_sensor_data, interval=10)


    plt.show()

    comms.stop()

    plt.close('all')


if __name__ == "__main__":
    main()
