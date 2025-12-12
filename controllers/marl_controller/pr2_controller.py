import numpy as np
from controller import Robot, Supervisor
import math
import sys

TIME_STEP = 16

MAX_WHEEL_SPEED = 8
WHEELS_DISTANCE = 0.4492
SUB_WHEELS_DISTANCE = 0.098
WHEEL_RADIUS = 0.08

KP_GAIN = 0.2
ANGLE_TOLERANCE_RAD = np.deg2rad(0.1)
MIN_SPEED = 2

TOLERANCE = 0.05
def ALMOST_EQUAL(a, b):
    return (a < b + TOLERANCE) and (a > b - TOLERANCE)


FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL = range(8)
FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION = range(4)
SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL = range(5)
LEFT_FINGER, RIGHT_FINGER = 0, 1

# robot = Robot()
robot = Supervisor()

wheel_motors = [None] * 8
wheel_sensors = [None] * 8
rotation_motors = [None] * 4
rotation_sensors = [None] * 4
left_arm_motors = [None] * 5
left_arm_sensors = [None] * 5
right_arm_motors = [None] * 5
right_arm_sensors = [None] * 5

left_finger_motor = None
left_finger_sensor = None
right_finger_motor = None
right_finger_sensor = None
left_finger_contact_sensors = [None, None]
right_finger_contact_sensors = [None, None]

head_tilt_motor = None
torso_motor = None
torso_sensor = None

imu_sensor = None
wide_stereo_l = None
wide_stereo_r = None
high_def = None
r_forearm = None
l_forearm = None
laser_tilt = None
base_laser = None


def step():
    if robot.step(TIME_STEP) == -1:
        sys.exit(0)

def initialize_devices():
    global left_finger_motor, left_finger_sensor
    global right_finger_motor, right_finger_sensor
    global head_tilt_motor, torso_motor, torso_sensor
    global imu_sensor, wide_stereo_l, wide_stereo_r, high_def
    global r_forearm, l_forearm, laser_tilt, base_laser

    names = [
        "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"
    ]
    for i in range(8):
        wheel_motors[i] = robot.getDevice(names[i])
        wheel_sensors[i] = wheel_motors[i].getPositionSensor()

    rotation_names = [
        "fl_caster_rotation_joint", "fr_caster_rotation_joint",
        "bl_caster_rotation_joint", "br_caster_rotation_joint"
    ]
    for i in range(4):
        rotation_motors[i] = robot.getDevice(rotation_names[i])
        rotation_sensors[i] = rotation_motors[i].getPositionSensor()

    left_arm_names = [
        "l_shoulder_pan_joint", "l_shoulder_lift_joint",
        "l_upper_arm_roll_joint", "l_elbow_flex_joint",
        "l_wrist_roll_joint"
    ]
    for i in range(5):
        left_arm_motors[i] = robot.getDevice(left_arm_names[i])
        left_arm_sensors[i] = left_arm_motors[i].getPositionSensor()

    right_arm_names = [
        "r_shoulder_pan_joint", "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint", "r_elbow_flex_joint",
        "r_wrist_roll_joint"
    ]
    for i in range(5):
        right_arm_motors[i] = robot.getDevice(right_arm_names[i])
        right_arm_sensors[i] = right_arm_motors[i].getPositionSensor()

    left_finger_motor = robot.getDevice("l_finger_gripper_motor::l_finger")
    left_finger_sensor = left_finger_motor.getPositionSensor()
    right_finger_motor = robot.getDevice("r_finger_gripper_motor::l_finger")
    right_finger_sensor = right_finger_motor.getPositionSensor()

    left_finger_contact_sensors[LEFT_FINGER] = robot.getDevice("l_gripper_l_finger_tip_contact_sensor")
    left_finger_contact_sensors[RIGHT_FINGER] = robot.getDevice("l_gripper_r_finger_tip_contact_sensor")
    right_finger_contact_sensors[LEFT_FINGER] = robot.getDevice("r_gripper_l_finger_tip_contact_sensor")
    right_finger_contact_sensors[RIGHT_FINGER] = robot.getDevice("r_gripper_r_finger_tip_contact_sensor")

    try:
        head_tilt_motor = robot.getDevice("head_tilt_joint")
        torso_motor = robot.getDevice("torso_lift_joint")
        torso_sensor = robot.getDevice("torso_lift_joint_sensor")
        imu_sensor = robot.getDevice("imu_sensor")
        wide_stereo_l = robot.getDevice("wide_stereo_l_stereo_camera_sensor")
        wide_stereo_r = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
        high_def = robot.getDevice("high_def_sensor")
        r_forearm = robot.getDevice("r_forearm_cam_sensor")
        l_forearm = robot.getDevice("l_forearm_cam_sensor")
        laser_tilt = robot.getDevice("laser_tilt")
        base_laser = robot.getDevice("base_laser")
    except:
        pass


def enable_devices():
    for i in range(8):
        wheel_sensors[i].enable(TIME_STEP)
        wheel_motors[i].setPosition(float('inf'))
        wheel_motors[i].setVelocity(0)

    for i in range(4):
        rotation_sensors[i].enable(TIME_STEP)

    for i in range(2):
        left_finger_contact_sensors[i].enable(TIME_STEP)
        right_finger_contact_sensors[i].enable(TIME_STEP)

    left_finger_sensor.enable(TIME_STEP)
    right_finger_sensor.enable(TIME_STEP)

    for i in range(5):
        left_arm_sensors[i].enable(TIME_STEP)
        right_arm_sensors[i].enable(TIME_STEP)

    try:
        torso_sensor.enable(TIME_STEP)
        base_laser.enable(TIME_STEP)
        imu_sensor.enable(TIME_STEP)
    except:
        pass


def set_wheels_speeds(*speeds):
    for i in range(8):
        wheel_motors[i].setVelocity(speeds[i])

def set_wheels_speed(speed):
    set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed)

def stop_wheels():
    set_wheels_speed(0)


torques = [0.0] * 8

def enable_passive_wheels(enable):
    if enable:
        for i in range(8):
            torques[i] = wheel_motors[i].getAvailableTorque()
            wheel_motors[i].setAvailableTorque(0)
    else:
        for i in range(8):
            wheel_motors[i].setAvailableTorque(torques[i])


def set_rotation_wheels_angles(fl, fr, bl, br, wait_on_feedback):
    if wait_on_feedback:
        stop_wheels()
        enable_passive_wheels(True)

    targets = [fl, fr, bl, br]
    for i in range(4):
        rotation_motors[i].setPosition(targets[i])

    if wait_on_feedback:
        while True:
            reached = True
            for i in range(4):
                if not ALMOST_EQUAL(rotation_sensors[i].getValue(), targets[i]):
                    reached = False
                    break
            if reached:
                break
            step()

        enable_passive_wheels(False)


def robot_rotate(angle):
    stop_wheels()
    set_rotation_wheels_angles(3 * math.pi / 4, math.pi / 4, -3 * math.pi / 4, -math.pi / 4, True)

    speed = MAX_WHEEL_SPEED if angle > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(speed)

    if imu_sensor:
        imu_roll, imu_pitch, imu_yaw = imu_sensor.getRollPitchYaw()
        initial_heading = imu_yaw
        target_heading = initial_heading + angle

        while True:
            imu_roll, imu_pitch, imu_yaw = imu_sensor.getRollPitchYaw()
            current_heading = imu_yaw
            error = target_heading - current_heading

            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi

            if abs(error) <= ANGLE_TOLERANCE_RAD:
                break
            speed = KP_GAIN * error
            speed = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speed))

            if abs(speed) < MIN_SPEED:
                speed = MIN_SPEED * np.sign(error)
            set_wheels_speed(speed)
            step()

    set_rotation_wheels_angles(0, 0, 0, 0, True)
    stop_wheels()


def robot_go_forward(dist):
    speed = MAX_WHEEL_SPEED if dist > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(speed)
    init_pos = wheel_sensors[FLL_WHEEL].getValue()

    while True:
        pos = wheel_sensors[FLL_WHEEL].getValue()
        travel = abs(WHEEL_RADIUS * (pos - init_pos))

        if travel > abs(dist):
            break
        if abs(dist) - travel < 0.025:
            set_wheels_speed(0.1 * speed)
        step()

    stop_wheels()


def set_gripper(left, open_gripper, torqueWhenGripping, wait):
    motor = left_finger_motor if left else right_finger_motor
    sensor = left_finger_sensor if left else right_finger_sensor
    contacts = left_finger_contact_sensors if left else right_finger_contact_sensors

    if open_gripper:
        target = 0.5
        motor.setPosition(target)
        if wait:
            while not ALMOST_EQUAL(sensor.getValue(), target):
                step()

    else:
        target = 0.0
        motor.setPosition(target)

        if wait:
            while ((contacts[0].getValue() == 0 or contacts[1].getValue() == 0) and
                   not ALMOST_EQUAL(sensor.getValue(), target)):
                step()

            current = sensor.getValue()
            motor.setAvailableTorque(torqueWhenGripping)
            motor.setPosition(max(0.0, 0.95 * current))


def set_right_arm_position(sR, sL, uR, eL, wR, wait):
    motors = right_arm_motors
    sensors = right_arm_sensors
    targets = [sR, sL, uR, eL, wR]
    for i in range(5):
        motors[i].setPosition(targets[i])

    if wait:
        while any(not ALMOST_EQUAL(sensors[i].getValue(), targets[i]) for i in range(5)):
            step()


def set_left_arm_position(sR, sL, uR, eL, wR, wait):
    motors = left_arm_motors
    sensors = left_arm_sensors
    targets = [sR, sL, uR, eL, wR]
    for i in range(5):
        motors[i].setPosition(targets[i])

    if wait:
        while any(not ALMOST_EQUAL(sensors[i].getValue(), targets[i]) for i in range(5)):
            step()


def set_torso_height(h, wait):
    if torso_motor:
        torso_motor.setPosition(h)
        if wait:
            while not ALMOST_EQUAL(torso_sensor.getValue(), h):
                step()


def set_initial_position():
    set_left_arm_position(0, 1.35, 0, -2.2, 0, False)
    set_right_arm_position(0, 1.35, 0, -2.2, 0, False)
    set_gripper(False, True, 0, False)
    set_gripper(True, True, 0, False)
    set_torso_height(0.2, True)