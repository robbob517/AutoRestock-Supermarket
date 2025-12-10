"""marl_controller controller."""
# Integrating code from path_planner.py and utilising PR2_qlearn_agent

# Adjusting python import root
import sys
import os

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.append(project_root)

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
    agent = qlearn.PR2_qlearn_Agent(robot, TIMESTEP, 0.5)

    robot_state = STATE_IDLE
    target_item = None
    current_action_index = None
    last_state_tuple = None
    previous_inventory = None

    # Main loop
    while robot.step(TIMESTEP) != -1:

        print(f"{robot_name}: Current state: {robot_state}")

        # Odometry
        delta_trans, delta_rot, current_x, current_y, current_theta, prev_wheels_angle = odometry.calc_odometry(current_x, current_y, current_theta, prev_wheels_angle)

        # Listen for server msgs
        server_data = None
        while receiver.getQueueLength() > 0:
            msg = receiver.getString()
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
            global_reward = server_data["reward"]

            # Update the q table

            # Calculate local reward
            success= True
            local_reward = agent.calculate_reward(previous_inventory, current_action_index, success)

            agent.update_q_table(last_state_tuple, current_action_index, local_reward, global_reward, new_state_tuple)
            last_state_tuple = None
            current_action_index = None

        # Choosing a new action
        if robot_state == STATE_IDLE and server_data:
            agent.inventory = server_data["inventory"]
            current_state_tuple = agent.get_state_tuple()

            current_action_index = agent.choose_action(current_state_tuple)
            target_item = agent.ACTIONS[current_action_index]

            target_coords = agent.ITEM_properties[target_item]["coords"]

            previous_inventory = agent.inventory.copy()
            last_state_tuple = current_state_tuple
            robot_state = STATE_PLANNING

            print(f"{robot_name}: Chosen item {target_item}")


        # Calculate path to goal
        elif robot_state == STATE_PLANNING:
            start_node = (current_x, current_y)
            goal_node = target_coords

            print(f"{robot_name}: Start Node -> Goal node: {start_node} -> {goal_node}")

            path = a_star_search.a_star_path(start_node, goal_node)

            if path:
                world_path = [test_map.map_to_world(cell[0], cell[1]) for cell in path]
                instructions = a_star_search.move_instructions(world_path)
                path_index = 0
                robot_state = STATE_MOVING
            else:
                robot_state = STATE_IDLE

        # Move to goal
        elif robot_state == STATE_MOVING:
            if path_index >= len(instructions):
                pr2.set_wheels_speed(0)
                robot_state = STATE_RESTOCKING
                continue

            target_dir, target_x, target_y = instructions[path_index]

            angle_to_turn = a_star_search.get_rotation(current_orientation, target_dir)
            if abs(angle_to_turn) > 1e-3:
                pr2.set_wheels_speed(0)
                current_orientation = target_dir

            if target_dir in ['up', 'down']:
                distance = abs(target_x - current_x)
            else:
                distance = abs(target_y - current_y)

            if distance > 0.01:
                pr2.set_wheels_speed(3)
                # If target_dir is left, right means robot needs to turn
            else:
                pr2.set_wheels_speed(0)
                path_index += 1

        elif robot_state == STATE_RESTOCKING:
            print(f"Robot {robot_name} restocking item {target_item}")
            msg = {
                "type" : "RESTOCK",
                "robot_id" : robot_name,
                "item" : target_item,
            }
            emitter.send(json.dumps(msg))

            robot_state = STATE_IDLE

run()