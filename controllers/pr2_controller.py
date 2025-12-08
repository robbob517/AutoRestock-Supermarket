import numpy as np
from controller import Robot
from controller import Supervisor
import math
import sys
import random
import pickle
import os

TIME_STEP = 32

MAX_WHEEL_SPEED = 3.0
WHEELS_DISTANCE = 0.4492
SUB_WHEELS_DISTANCE = 0.098
WHEEL_RADIUS = 0.08

KP_GAIN = 0.05
ANGLE_TOLERANCE_RAD = np.deg2rad(1.0)
MIN_SPEED = 0.5

TOLERANCE = 0.05
def ALMOST_EQUAL(a, b):
    return (a < b + TOLERANCE) and (a > b - TOLERANCE)


FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL = range(8)
FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION = range(4)
SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL = range(5)
LEFT_FINGER, RIGHT_FINGER = 0, 1

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



class PR2_qlearn_Agent:
    def __init__(self):

        self.ALPHA =0.1
        self.GAMMA =0.9
        self.EPSILON =1.0
        self.MIN_EPSILON =0.05
        self.DECAY =0.999

        self.ITEM_PROPERTIES = {
            "ICE_CREAM": {"dist": 12.0, "type": "FROZEN", "start": 2},
            "PIZZA": {"dist": 10.0, "type": "FROZEN", "start": 2},
            "CHICKEN": {"dist": 11.0, "type": "FROZEN", "start": 1},
            "PEAS": {"dist": 10.0, "type": "FROZEN", "start": 0},
            "MILK": {"dist": 5.0, "type": "PRODUCE", "start": 3},
            "APPLES": {"dist": 4.0, "type": "PRODUCE", "start": 5},
            "BANANAS": {"dist": 4.0, "type": "PRODUCE", "start": 2},
            "EGGS": {"dist": 5.0, "type": "PRODUCE", "start": 2},
            "YOGURT": {"dist": 5.5, "type": "PRODUCE", "start": 3},
            "BREAD": {"dist": 2.0, "type": "DRY", "start": 4},
            "CEREAL": {"dist": 3.0, "type": "DRY", "start": 0},
            "WATER": {"dist": 8.0, "type": "DRY", "start": 4},
            "CHIPS": {"dist": 2.0, "type": "DRY", "start": 1},
            "PASTA": {"dist": 3.0, "type": "DRY", "start": 7},
            "SOAP": {"dist": 9.0, "type": "NON_FOOD", "start": 2}
        }
        #dist is distance of robot from shelf that needs item
        #type is the category of the item
        #start is how much of the item is already in stock at the start of the stocking process

        self.ACTIONS = list(self.ITEM_PROPERTIES.keys())
        self.NUM_ACTIONS = len(self.ACTIONS)
        self.q_table_file ="pr2_q_memory.pkl"
        self.q_table =self.load_q_table()

        self.inventory = {k: v["start"] for k, v in self.ITEM_PROPERTIES.items()}

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


    def load_q_table(self):
            if os.path.exists(self.q_table_file):
                with open(self.q_table_file,'rb') as f:
                    return pickle.load(f)
            else:
                return {}

    def save_q_table(self):
            print("saving q table")
            with open(self.q_table_file,'wb') as f:
                pickle.dump(self.q_table,f)

    # implementing a system where amount of stocked items is categorized into 4 levels so that for larger sets q table doesnt need to grow exponentially
    def get_discrete_level(self, count):
        if count <= 5:
            return 0  #Bad
        elif count <= 14:
            return 1  #OK
        elif count <= 19:
            return 2  #Good
        else:
            return 3 #Stocked (Full)

    def get_state_tuple(self):
        current_levels = []
        for item in self.ACTIONS:
            raw_count =self.inventory[item]
            state_cat =self.get_discrete_level(raw_count)
            current_levels.append(state_cat)
        return tuple(current_levels)

    def get_q_values(self, state):
        if state not in self.q_table:
            self.q_table[state] = np.zeros(self.NUM_ACTIONS)
        return self.q_table[state]

    def choose_action(self, state):
        if random.uniform(0, 1) < self.EPSILON:
            return random.randint(0,self.NUM_ACTIONS - 1)
        else:
            q_vals = self.get_q_values(state)
            return np.argmax(q_vals)

    def execute_action_text_mode(self, action_idx, step_num):
        # text mode to check if q table is running well ( not really needed for actual robot)
        item_name= self.ACTIONS[action_idx]
        props= self.ITEM_PROPERTIES[item_name]
        print(f"   [Step {step_num} | Eps: {self.EPSILON:.4f}] Stocking {item_name} (Type: {props['type']})...")
        if self.inventory[item_name] <20:
            self.inventory[item_name]+= 1
            new_level = self.inventory[item_name]
            status = "CRITICAL" if new_level <= 5 else "OK" if new_level <= 14 else "GOOD" if new_level <= 19 else "FULL"
            print(f"   [RESULT] Level: {new_level}/20 ({status})")
            return True
        else:
            print(f"   [FAIL] {item_name} is already at 20.")
            return False

    def calculate_reward(self, old_inv, action_idx, success):
        item_name = self.ACTIONS[action_idx]
        props = self.ITEM_PROPERTIES[item_name]
        old_count = old_inv[item_name]
        old_cat = self.get_discrete_level(old_count)

        base_reward = 0

        if old_cat == 0:
            base_reward= 80
        elif old_cat == 1:
            base_reward= 40
        elif old_cat == 2:
            base_reward= 10
        elif old_cat == 3:
            base_reward= -20

        # multiplier is higher if frozen goods are done first , then produce then non food since no real time constraint

        multiplier= 1.0
        if props['type'] == "FROZEN":
            multiplier= 2.0
        elif props['type'] == "PRODUCE":
            multiplier= 1.5
        elif props['type'] == "NON_FOOD":
            multiplier= 0.5

        final_stock_reward = base_reward * multiplier
        distance_penalty = props['dist'] * 0.5
        total_reward = final_stock_reward - distance_penalty

        if not success and old_count != 20:
            total_reward -= 5

        return total_reward

    def update_q_table(self,s,a,r,s_prime):
        q_values = self.get_q_values(s)
        old_q = q_values[a]
        next_max = np.max(self.get_q_values(s_prime))
        new_q = old_q + self.ALPHA * (r + self.GAMMA * next_max - old_q)
        self.q_table[s][a] = new_q

    def train_text_mode(self):
        # used to check if the q table is working correctly will be removed when the robot works on its own
        print(f"Starting Q-Learning ")
        episode = 0
        steps_this_day = 0
        while robot.step(TIME_STEP) != -1:
            episode+=1
            steps_this_day+= 1

            state =self.get_state_tuple()
            old_inv = self.inventory.copy()
            action_idx = self.choose_action(state)
            success =self.execute_action_text_mode(action_idx, steps_this_day)

            reward = self.calculate_reward(old_inv, action_idx, success)
            next_state = self.get_state_tuple()
            self.update_q_table(state,action_idx,reward,next_state)

            if self.EPSILON > self.MIN_EPSILON:
                self.EPSILON *= self.DECAY

            if all(value == 20 for value in self.inventory.values()):
                print(f"\n Store fully stocked")

                self.save_q_table()

                if steps_this_day < 350:
                    break
                self.inventory = {k: v["start"] for k, v in self.ITEM_PROPERTIES.items()}
                steps_this_day = 0

if __name__ == "__main__":
    initialize_devices()
    enable_devices()

    agent = PR2_qlearn_Agent()
    agent.train_text_mode()()