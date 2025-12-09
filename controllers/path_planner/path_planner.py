from controller import Robot
import numpy as np
import math
import time
import signal
import sys
import helper
import random
import odometry
import heapq
import a_star_search
import particle_filtering
import test_map

# ----------------------
# Simulation / robot params
# ----------------------
TIME_STEP = 32

# Particle Filtering
num_particles = 200

# Odometry
prev_wheels_angle = np.zeros(8)
delta_trans, delta_rot, x, y, theta = 0.0, 0.0, -3.0, -12.0, 0.0
total_distance = 0

# Lidar
#lidar = helper.base_laser
Z_HIT = 0.95
Z_RAND = 0.05
SIGMA = 0.2     # noise in meters

# Function to display particles on webots world in realtime
root = helper.robot.getRoot()
children_field = root.getField("children")
particle_nodes = []

def draw_particles(particles):
    global particle_nodes
    # Remove old particles
    for node_index in reversed(particle_nodes):
        children_field.removeMF(node_index)
    particle_nodes.clear()

    # Add new particles as small spheres
    for p in particles:
        x, z = p["x"], p["y"]
        # Color by weight (optional)
        weight = p.get("weight", 0.05)
        color = (1.0, max(0, 1 - weight*5), 0)  # brighter red for higher weight
        particle_str = f"""
        Transform {{
            translation {x} 0.05 {z}
            children [
                Shape {{
                    appearance Appearance {{
                        material Material {{ diffuseColor {color[0]} {color[1]} {color[2]} }}
                    }}
                    geometry Sphere {{ radius 0.02 }}
                }}
            ]
        }}
        """
        index = children_field.getCount()
        children_field.importMFNodeFromString(index, particle_str)
        particle_nodes.append(index)



"Initialising the devices"
helper.initialize_devices()
helper.enable_devices()
helper.set_initial_position()

start = (-3, -12)
goal = None
path_index = 0
current_orientation = 'up'
target_orientation = None
goal_commands = [(0.25, 10), (-4.2, 1), (0, 4), (-2, -4.2), (4, 4)]#, (-1, 0), (4.2, -3), (-4, 2), (1, 1), (-3, -4)]
command_index = 0

particles = particle_filtering.initialize_particle(test_map.og, num_particles, (0, 0, 0))
print(test_map.og)

while helper.robot.step(TIME_STEP) != -1:

    # Check for new goal nodes when reached to the previous one
    if goal is None:
        if command_index == len(goal_commands):
            print("Goal Commands reached")
            exit(-1)
        goal = goal_commands[command_index]
        command_index += 1
        '''
        goal_input = input("Enter goal node x and y or exit: ")
        if goal_input.lower() == "exit":
            print("Exiting program.")
            exit(-1)
        goal = tuple(map(int, goal_input.split()))
        '''
        print("Goal:", goal)
        print("Start", start)
        # Path finder to the goal node
        path = a_star_search.a_star_path(start, goal)
        world_path = [test_map.map_to_world(cell[0], cell[1]) for cell in path]
        print(world_path)
        # Obtain move instructions for the robot  according to path
        instructions = a_star_search.move_instructions(world_path)
        #print(instructions)
        '''
        lidar = helper.base_laser.getRangeImage()
        if lidar[len(lidar) // 2] <= 0.3:
            helper.set_wheels_speed(0)

            delta_trans, delta_rot, x, y, theta, prev_wheels_angle = odometry.calc_odometry(x, y, theta,
                                                                                            prev_wheels_angle)
            start = [x, y]
        '''


    # Update odometry
    delta_trans, delta_rot, x, y, theta, prev_wheels_angle = odometry.calc_odometry(x,y,theta,prev_wheels_angle)
    print("x: ", x, "y: ", y, "theta: ", theta)

    # Particle Filter
    '''
    particles = particle_filtering.motion_update(particles, odometry.calc_odometry(x,y,theta,prev_wheels_angle), noise_std=[0.01, 0.01, 0.005])
    particles = particle_filtering.cal_particle_weight(particles, test_map.og)
    particles = particle_filtering.resample(particles)
    est_x, est_y, est_theta = particle_filtering.estimate_pose(particles)
    #print("x: ", est_x, "y: ", est_y, "theta: ", est_theta)
    #print([particle["weight"] for particle in particles])
    #straight, left, right = particle_filtering.lidar_readings()
    #print("Straight: ", straight, "Left: ", left, "Right: ", right)
    #print("x: ", x, ", y: ", y, ", theta: ", theta)
    '''
    print("Working")
    # Main algorithm for robot movement
    if path_index >= len(instructions):
        helper.set_wheels_speed(0)
        print("Reached goal!", goal)
        start = goal
        goal = None
        path_index = 0
        continue

    target_dir, target_x, target_y = instructions[path_index]

    # Rotate robot according to target direction
    angle_to_turn = a_star_search.get_rotation(current_orientation, target_dir)
    if abs(angle_to_turn) > 1e-3:
        helper.set_wheels_speed(0)
        helper.robot_rotate(angle_to_turn)  # blocking call, rotates robot
        current_orientation = target_dir

    #  Move robot forward
    if target_dir in ['up', 'down']:
        distance = abs(target_x - x)
    else:
        distance = abs(target_y - y)

    if distance > 0.01:
        helper.set_wheels_speed(3)
    # If target_dir is left, right means robot needs to turn
    else:
        helper.set_wheels_speed(0)
        path_index += 1
        #print("path_index: ", path_index)

