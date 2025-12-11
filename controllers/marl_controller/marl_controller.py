import json
import numpy as np

import pr2_controller as pr2
import pr2_qlearn_agent as qlearn
import odometry as od
import a_star_search as pathfind

TIMESTEP = 16
COOP_WEIGHTING = 0.7

STATE_IDLE = 0
STATE_PLANNING = 1
STATE_MOVING = 2
STATE_REFILLING = 3
STATE_RESTOCKING = 4

def rotation_snap(robot_node):
    rotation_field = robot_node.getField("rotation")
    current_rotation = rotation_field.getSFRotation()

    z_axis = current_rotation[2]
    raw_yaw = current_rotation[3]

    if z_axis < 0:
        current_yaw = -raw_yaw
    else:
        current_yaw = raw_yaw

    snapped_yaw = round(current_yaw / (np.pi / 2)) * (np.pi / 2)

    rotation_field.setSFRotation([0, 0, 1, snapped_yaw])

    robot_node.resetPhysics()

def run():
    robot = pr2.robot
    robot_name = robot.getName()
    robot_node = pr2.robot.getFromDef(robot_name.upper())
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(TIMESTEP)

    pr2.initialize_devices()
    pr2.enable_devices()
    pr2.set_initial_position()

    robot_state = STATE_IDLE

    # Items that robot is holding (logical)
    robot_stock = 0
    other_robots = {}
    global_reward = 0

    # Q learning variable init
    agent = qlearn.PR2_qlearn_Agent(robot, TIMESTEP, COOP_WEIGHTING)

    target_item = None
    current_action_index = None
    last_state_tuple = None
    previous_inventory = None

    # Navigation variable init (Using ground truth, update with odometry)
    current_x = robot_node.getPosition()[0]
    current_y = robot_node.getPosition()[1]
    origin = (current_x, current_y)
    current_orientation = "up"
    path_index = 0

    # Unused
    current_theta = 1.5708
    prev_wheels_angle = np.zeros(8)

    print(f"{robot_name}: Initialised at position {current_x}, {current_y}")

    step = 0

    while robot.step(TIMESTEP) != -1:

        # step += 1
        # if (step % TIMESTEP*2) == 0:
        #     print(f"{robot_name}: Estimated position: {current_x}, {current_y}")

        # Odometry
        # delta_trans, delta_rot, current_x, current_y, current_theta, prev_wheels_angle = od.calc_odometry(
        #     current_x, current_y, current_theta, prev_wheels_angle)

        # Ground Truth for position
        ground_position = robot_node.getPosition()
        current_x = ground_position[0]
        current_y = ground_position[1]

        # Listen for server messages
        data = None
        while receiver.getQueueLength() > 0:
            msg = receiver.getString()
            data = json.loads(msg)

            if data["type"] == "GLOBAL":
                global_reward = data["reward"]
                other_robots = data["robots_positions"]
                previous_inventory = data["inventory"]

            receiver.nextPacket()

        # Update server with position
        robot_update = {"type" : "UPDATE_POS",
                        "robot_id" : robot_name,
                        "position" : (current_x, current_y),
                        "orientation" : current_orientation,
                        "state" : robot_state,
                        }
        emitter.send(json.dumps(robot_update))

        # If no data received, skip
        if data is None and robot_state == STATE_IDLE:
            continue

        # Update Q learning
        if data and last_state_tuple is not None and current_action_index is not None:
            agent.inventory = previous_inventory
            new_state_tuple = agent.get_state_tuple()

            # Calculate local reward
            success = True
            local_reward = agent.calculate_reward(previous_inventory, current_action_index, success)

            agent.update_q_table(last_state_tuple, current_action_index, local_reward, global_reward, new_state_tuple)

            last_state_tuple = None
            current_action_index = None

        # Marl Loop
        # Choose a new action
        if robot_state == STATE_IDLE:
            if robot_stock > 0:
                agent.inventory = previous_inventory
                current_state_tuple = agent.get_state_tuple()
                current_action_index = agent.choose_action(current_state_tuple)

                target_item = agent.ACTIONS[current_action_index]
                target_coords = agent.ITEM_properties[target_item]["coords"]

                previous_inventory = agent.inventory.copy()
                last_state_tuple = current_state_tuple
                robot_state = STATE_PLANNING

                print(f"{robot_name}: Chosen item {target_item} at {target_coords}")

            else:
                print(f"{robot_name}: Inventory empty, going to refill")
                robot_state = STATE_PLANNING
                target_coords = origin

        # Calculating path to goal position
        elif robot_state == STATE_PLANNING:
            origin_x, origin_y = origin
            if (np.sqrt((current_x - origin_x) ** 2 + (current_y - origin_y) ** 2) < 1) and robot_stock == 0:
                robot_state = STATE_REFILLING
            else:
                start_node = (current_x, current_y)
                goal_node = target_coords

                print(f"{robot_name}: Start Node -> Goal node: {start_node} -> {goal_node}")

                path = pathfind.a_star_path(start_node, goal_node)

                if path:
                    instructions = pathfind.move_instructions(path)
                    robot_state = STATE_MOVING
                else:
                    print(f"{robot_name}: Path not found to {target_item}")
                    robot_state = STATE_IDLE

        # Move to goal position
        elif robot_state == STATE_MOVING:
            if path_index >= len(instructions):
                pr2.set_wheels_speed(0)
                path_index = 0
                if robot_stock > 0:
                    robot_state = STATE_RESTOCKING
                else:
                    robot_state = STATE_REFILLING
                continue

            target_dir, target_x, target_y = instructions[path_index]

            # Basic collision avoidance with other robots (Waits while other robot is in its goal node)
            path_blocked = False
            for other_robot in other_robots.keys():
                if other_robot != robot_name:
                    # Calculate distance of other robot to current robot's goal node
                    other_dist = np.sqrt((target_x - other_robots[other_robot]['position'][0]) ** 2 + (target_y - other_robots[other_robot]['position'][1]) ** 2)

                    # Calculate distance of current robot to goal node
                    current_dist = np.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

                    if other_dist < 1.5 and current_dist < 2:
                        path_blocked = True
                        break

            if path_blocked:
                pr2.set_wheels_speed(0)
                continue

            angle_to_turn = pathfind.get_rotation(current_orientation, target_dir)
            if abs(angle_to_turn) > 1e-3:
                pr2.set_wheels_speed(0)
                pr2.robot_rotate(angle_to_turn)

                rotation_snap(robot_node)

                current_orientation = target_dir

            if target_dir in ['up', 'down']:
                distance = abs(target_y - current_y)
            else:
                distance = abs(target_x - current_x)

            if distance > 0.01:
                pr2.set_wheels_speed(pr2.MAX_WHEEL_SPEED)
                # If target_dir is left, right means robot needs to turn
            else:
                pr2.set_wheels_speed(0)
                path_index += 1

        # Return to storage area to refill robot's inventory
        elif robot_state == STATE_REFILLING:
            previous_stock = robot_stock
            robot_stock = 2

            print(f"{robot_name}: Stock refilled from {previous_stock} to {robot_stock}")

            robot_state = STATE_IDLE

        # Send restocking instruction to supermarket manager
        elif robot_state == STATE_RESTOCKING:
            print(f"{robot_name}: Restocking item {target_item} at {target_coords}")
            restock_msg = {
                "type" : "RESTOCKING",
                "robot_id" : robot_name,
                "item_name" : target_item,
                "item_pos" : target_coords,
            }
            emitter.send(json.dumps(restock_msg))
            robot_stock -= 1
            robot_state = STATE_IDLE


run()

