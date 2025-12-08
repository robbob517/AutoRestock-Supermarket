import math
import random
import numpy as np
import test_map
import helper

# Function to initialize particles. If robot start position is know then around it otherwise at random
def initialize_particle(map_grid, num_particles, known_start=None):
    particles = []
    weight = 1.0 / num_particles

    # OPTION A: Known Start Position
    if known_start is not None:
        start_x, start_y, start_theta = known_start

        initial_noise_xy = 0.05
        initial_noise_theta = 0.1

        for _ in range(num_particles):
            particles.append({
                "x": start_x + np.random.normal(0, initial_noise_xy),
                "y": start_y + np.random.normal(0, initial_noise_xy),
                "theta": start_theta,
                "weight": weight
            })

    # OPTION B: Unknown Start Position
    else:
        free_x, free_y = np.where(map_grid == 1)

        if len(free_x) == 0:
            raise ValueError("No free cells found in map. Cannot initialize particles.")

        indices = np.random.choice(len(free_x), size=num_particles)

        for idx in indices:
            mx = free_x[idx]
            my = free_y[idx]

            wx, wy = test_map.map_to_world(mx, my)

            theta = random.choice([0, -math.pi, math.pi / 2, -math.pi / 2])

            particles.append({
                "x": wx,
                "y": wy,
                "theta": theta,
                "weight": weight
            })

    return particles

# Function to update particles position according to robot movement using odometry.
def motion_update(particles, odom, noise_std):
    delta_trans = odom[0]
    delta_rot = odom[1]

    for p in particles:
        noisy_trans = delta_trans + np.random.normal(0, noise_std[0])
        noisy_rot = delta_rot + np.random.normal(0, noise_std[2]) # Assuming index 2 is for rotation


        p["x"] += noisy_trans * math.cos(p["theta"])
        p["y"] += noisy_trans * math.sin(p["theta"])

        p["theta"] += noisy_rot

        #Normalize angle (Keep it between -pi and pi)
        p["theta"] = (p["theta"] + math.pi) % (2 * math.pi) - math.pi

    return particles

# Function to get three lidar reading in different directions
def lidar_readings():
    lidar_scans = helper.base_laser.getRangeImage()
    straight = lidar_scans[320]
    left = lidar_scans[541]
    right = lidar_scans[99]
    # These added values is to consider robot body
    return straight+0.32630539, left+0.04990911, right+0.0499773


def cal_particle_weight(particles, map):
    world_map = map
    lidar_straight, lidar_left, lidar_right = lidar_readings()
    new_particles = []
    for particle in particles:
        particle_weight = 0
        particle_x = particle['x']
        particle_y = particle['y']
        particle_theta = particle['theta']
        # Particle facing straight
        if -0.785 <= particle_theta < 0.785:
            obsticle_straight = particle_x + lidar_straight
            obsticle_left = particle_y + lidar_left
            obsticle_right = particle_y - lidar_right

            obsticle_check1 = test_map.world_to_map(obsticle_straight, particle_y)
            obsticle_check2 = test_map.world_to_map(particle_x, obsticle_left)
            obsticle_check3 = test_map.world_to_map(particle_x, obsticle_right)
        # Particle facing right
        elif -2.356 <= particle_theta < -0.785:
            obsticle_straight = particle_y - lidar_straight
            obsticle_left = particle_x + lidar_left
            obsticle_right = particle_x - lidar_right

            obsticle_check1 = test_map.world_to_map(particle_x, obsticle_straight)
            obsticle_check2 = test_map.world_to_map(obsticle_left, particle_y)
            obsticle_check3 = test_map.world_to_map(obsticle_right, particle_y)
        # Particle facing back
        elif particle_theta >= 2.356 or particle_theta < -2.356:
            obsticle_straight = particle_x - lidar_straight
            obsticle_left = particle_y - lidar_left
            obsticle_right = particle_y + lidar_right

            obsticle_check1 = test_map.world_to_map(obsticle_straight, particle_y)
            obsticle_check2 = test_map.world_to_map(particle_x, obsticle_left)
            obsticle_check3 = test_map.world_to_map(particle_x, obsticle_right)
        # Particle facing left
        elif 0.785 <= particle_theta < 2.356:
            obsticle_straight = particle_y + lidar_straight
            obsticle_left = particle_x - lidar_left
            obsticle_right = particle_x + lidar_right

            obsticle_check1 = test_map.world_to_map(particle_x, obsticle_straight)
            obsticle_check2 = test_map.world_to_map(obsticle_left, particle_y)
            obsticle_check3 = test_map.world_to_map(obsticle_right, particle_y)

        '''
        if 0 <= obsticle_check1[0] < 200  and 0 <= obsticle_check1[1] < 200:
            if world_map[ int(obsticle_check1[0]) ][ int(obsticle_check1[1]) ] == 2:
                particle_weight += 1
        if 0 <= obsticle_check2[0] < 200 and 0 <= obsticle_check2[1] < 200:
            if world_map[ int(obsticle_check2[0]) ][ int(obsticle_check2[1]) ] == 2:
                particle_weight += 1
        if 0 <= obsticle_check3[0] < 200 and 0 <= obsticle_check3[1] < 200:
            if world_map[ int(obsticle_check3[0]) ][ int(obsticle_check3[1]) ] == 2:
                particle_weight +=1

        particle["weight"] = particle_weight
        '''
        match_quality1 = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = obsticle_check1[0] + dx, obsticle_check1[1] + dy
                if 0 <= nx < 200 and 0 <= ny < 200:
                    if world_map[nx][ny] == 2:
                        # If we found a wall nearby, give points!
                        # Center hit = 1.0 point
                        # Neighbor hit = 0.5 points
                        if dx == 0 and dy == 0:
                            match_quality1 = max(match_quality1, 1.0)
                        else:
                            match_quality1 = max(match_quality1, 0.5)

        match_quality2 = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = obsticle_check2[0] + dx, obsticle_check2[1] + dy
                if 0 <= nx < 200 and 0 <= ny < 200:
                    if world_map[nx][ny] == 2:
                        # If we found a wall nearby, give points!
                        # Center hit = 1.0 point
                        # Neighbor hit = 0.5 points
                        if dx == 0 and dy == 0:
                            match_quality2 = max(match_quality2, 1.0)
                        else:
                            match_quality2 = max(match_quality2, 0.5)

        match_quality3 = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = obsticle_check3[0] + dx, obsticle_check3[1] + dy
                if 0 <= nx < 200 and 0 <= ny < 200:
                    if world_map[nx][ny] == 2:
                        # If we found a wall nearby, give points!
                        # Center hit = 1.0 point
                        # Neighbor hit = 0.5 points
                        if dx == 0 and dy == 0:
                            match_quality3 = max(match_quality3, 1.0)
                        else:
                            match_quality3 = max(match_quality3, 0.5)

        particle["weight"] = (match_quality1 + match_quality2 + match_quality3) / 3

    return particles


def resample(particles):
    weights = [p['weight'] for p in particles]

    if sum(weights) == 0:
        print("Warning: All weights are zero! Skipping resampling.")
        return particles

    chosen_particles = random.choices(particles, weights=weights, k=len(particles))

    new_particles = []
    for p in chosen_particles:

        new_particle = p.copy()
        new_particle['weight'] = 1.0
        new_particles.append(new_particle)

    return new_particles


def estimate_pose(particles):
    # Initialize sums
    x_sum = 0.0
    y_sum = 0.0
    cos_sum = 0.0
    sin_sum = 0.0
    total_weight = 0.0

    for p in particles:
        w = p['weight']

        # Accumulate weighted sums
        x_sum += p['x'] * w
        y_sum += p['y'] * w

        # Handle Angle Averaging (Vector Math)
        cos_sum += math.cos(p['theta']) * w
        sin_sum += math.sin(p['theta']) * w

        total_weight += w

    # Safety Check: Avoid division by zero
    if total_weight == 0:
        return 0, 0, 0  # Or return the last known position

    # Calculate final averages
    est_x = x_sum / total_weight
    est_y = y_sum / total_weight
    est_theta = math.atan2(sin_sum, cos_sum)

    return est_x, est_y, est_theta




