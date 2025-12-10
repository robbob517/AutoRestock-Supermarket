# Integrating code from path_planner.py and utilising PR2_qlearn_agent

import json
import numpy as np

from controllers.path_planner import odometry, a_star_search, test_map
from controllers import pr2_qlearn_agent as qlearn
from controllers import pr2_controller as pr2

TIMESTEP = 16

STATE_IDLE = 0
STATE_PLANNING = 1
STATE_MOVING = 2
STATE_REFILLING = 3
STATE_RESTOCKING = 4

def run():
    robot = pr2.robot
    robot_name = robot.getName()
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(TIMESTEP)

    pr2.initialize_devices()
    pr2.enable_devices()
    pr2.set_initial_position()

    # Initialise path planning (from path_planner.py)
    if robot_name == "pr2_1":
        current_x, current_y, current_theta = 2, -12, 0
    else:
        current_x, current_y, current_theta = -2, -12, 0
    prev_wheels_angle = np.zeros(8)

    current_path = []
    path_index = 0
    current_orientation = 'up'

    # Initialise qlearning agent
    agent = qlearn.PR2_qlearn_Agent(robot, TIMESTEP)

    robot_state = STATE_IDLE
    target_item = None
    current_action_index = None
    last_state_tuple = None

    # Main loop
    while robot.step(TIMESTEP) != -1:
        # Odometry
        delta_trans, delta_rot, current_x, current_y, current_theta, prev_wheels_angle = odometry.calc_odometry(current_x, current_y, current_theta, prev_wheels_angle)

        # Listen for server msgs
        server_data = None
        while receiver.getQueueSize() > 0:
            msg = receiver.getMessage().decode('utf-8')
            data = json.loads(msg)
            receiver.nextPacket()
            if data["type"] == "STATE":
                server_data = data

        # Check for data received, if not, wait until received
        if server_data is None and robot_state == STATE_IDLE:
            continue

        # MARL Loop

        # Q-learning
        if server_data and last_state_tuple is not None and current_action_index is not None:
            agent.inventory = server_data["inventory"]
            new_state_tuple = agent.get_state_tuple()
            reward = server_data["reward"]

            # Update the q table
            agent.update_q_table(last_state_tuple, current_action_index, reward, new_state_tuple)
            last_state_tuple = None

        if robot_state == STATE_IDLE and server_data:
            agent.inventory = server_data["inventory"]
            current_state_tuple = agent.get_state_tuple()

            current_action_index = agent.choose_action(current_state_tuple)
            target_item = agent.ACTIONS[current_action_index]

            target_coords = agent.ITEM_properties[target_item]["coords"]

            robot_state = STATE_PLANNING
            last_state_tuple = current_state_tuple