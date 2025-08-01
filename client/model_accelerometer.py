import numpy as np

def get_x_rotation(theta):

    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

def get_y_rotation(theta):

    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

l = np.array([[1.253314, 0.000000, -0.000000],
[0.000000, 1.253314, -0.000000],
[0.000000, 0.000000, 1.253314]])

print(l)

print(l @ l.T)

# real measurement when have negative y rotatino of about pi /4: 8.331076, 0.041314, 4.772077
# real measurement when have positive x rotation of about pi /4: -0.096, 6.921607, 6.944359
# real measurement when have positive y rotation of about pi/4:  -8.805289, 0.052092, 4.531377
# real measurement when have negative x rotation of about pi/4: -0.077239, -8.041279, 5.506749

gravity = np.array([0, 0, 9.81])
def get_expected_measurement(state):

    return get_x_rotation(cur_rot[0]).transpose() @ (get_y_rotation(cur_rot[1]).transpose() @ gravity)
    
# for i in range(2):
#     for j in [-1, 1]:
#         cur_rot = np.array([-0.05, 0.03, -0.01])
#         cur_rot[i] = j * np.pi/4
#         print(cur_rot)
#         print(get_expected_measurement(cur_rot))
#         print()


cur_rot = np.array([np.pi, -np.pi, 0.0])
print(cur_rot)
print(get_expected_measurement(cur_rot))

# print()

# https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6172226
# out_meas = gravity[2] * np.array([-np.sin(cur_rot[1]), np.cos(cur_rot[1]) * np.sin(cur_rot[0]), np.cos(cur_rot[1]) * np.cos(cur_rot[0])])
# print(out_meas)