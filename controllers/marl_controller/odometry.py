import math

import numpy as np
from controllers.marl_controller import pr2_controller as pr2

# PR2 constants
WHEELS_DISTANCE = 0.4492
SUB_WHEELS_DISTANCE = 0.098
WHEEL_RADIUS = 0.08

# Global Robot Pose Values
x = 0.0
y = 0.0
theta = 0.0

prev_wheels_angle = np.zeros(8)

def calc_odometry(x, y, theta, prev_wheels_angle):
    current_wheels_angle = np.array([pr2.wheel_sensors[i].getValue() for i in range(8)])
    delta_wheels_angle = current_wheels_angle - prev_wheels_angle
    prev_wheels_angle[:] = current_wheels_angle

    # Wheel movement
    distance_per_wheel = delta_wheels_angle * WHEEL_RADIUS

    avg_distance_left = (distance_per_wheel[0] + distance_per_wheel[1] +
                         distance_per_wheel[4] + distance_per_wheel[5]) / 4
    avg_distance_right = (distance_per_wheel[2] + distance_per_wheel[3] +
                          distance_per_wheel[6] + distance_per_wheel[7]) / 4

    # Forward movement
    delta_trans = (avg_distance_left + avg_distance_right) / 2

    # Using IMU for rotation
    imu_roll, imu_pitch, imu_yaw = pr2.imu_sensor.getRollPitchYaw()

    # Change in rotation
    delta_rot = (imu_yaw - theta + math.pi) % (2 * math.pi) - math.pi

    if abs(avg_distance_left + avg_distance_right) < -0.5 or abs(avg_distance_left + avg_distance_right) > 0.5:
        return 0, 0, x, y, theta, prev_wheels_angle

    # Update real robot pose
    x_new = x + delta_trans * math.cos(theta)
    y_new = y + delta_trans * math.sin(theta)
    theta_new = imu_yaw

    return delta_trans, delta_rot, x_new, y_new, theta_new, prev_wheels_angle