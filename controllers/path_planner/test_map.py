import numpy as np

# Map params
MAP_RES = 0.05    # meters per cell
WORLD_SIZE = 30.0
MAP_SIZE = int(WORLD_SIZE / MAP_RES)
MAP_ORIGIN = MAP_SIZE // 2

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

SHELVES_X = 1
SHELVES_Y = 4.6

# Create empty map (all FREE)
og = np.full((MAP_SIZE, MAP_SIZE), FREE, dtype=int)

# Set walls on all edges
og[0, :] = OCCUPIED
og[-1, :] = OCCUPIED
og[:, 0] = OCCUPIED
og[:, -1] = OCCUPIED

def world_to_map(wx, wy):
    mx = int(round(wx / MAP_RES)) + int(MAP_ORIGIN)
    my = int(round(wy / MAP_RES)) + int(MAP_ORIGIN)
    return mx, my

def map_to_world(mx, my):
    wx = (mx - MAP_ORIGIN) * MAP_RES
    wy = (my - MAP_ORIGIN) * MAP_RES
    return wx, wy

def in_map(mx, my):
    return 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE

def add_objects(x, y, size_x, size_y):
    # Calculate corners in meters
    start_x_m = x - (size_x / 2)
    end_x_m = x + (size_x / 2)
    start_y_m = y - (size_y / 2)
    end_y_m = y + (size_y / 2)

    x1, y1 = world_to_map(start_x_m, start_y_m)
    x2, y2 = world_to_map(end_x_m, end_y_m)

    og[x1:x2, y1:y2,] = OCCUPIED

add_objects(10, 0, 30, 0.8) # East Wall
add_objects(-10, 0, 30, 0.8) # West Wall
add_objects(0, 14.9, 19.8, 0.8) # North Wall
add_objects(0, -14.9, 19.8, 0.8) # South Wall
add_objects(-5.65, -8, 10, 2) # Storage Divider Right
add_objects(5.65, -8, 10, 2) # Storage Divider Left
y1 = -3
for x in range(3):
    add_objects(-7.5, y1, SHELVES_X, SHELVES_Y)
    add_objects(-6.75, y1, SHELVES_X, SHELVES_Y)

    y1 += 4.01

y2 = -3
for x in range(3):
    add_objects(-3.75, y2, SHELVES_X, SHELVES_Y)
    add_objects(-3.25, y2, SHELVES_X, SHELVES_Y)
    y2 += 4.01

y3 = -3
for x in range(3):
    add_objects(-0.25, y3, SHELVES_X, SHELVES_Y)
    add_objects(0.25, y3, SHELVES_X, SHELVES_Y)
    y3 += 4.01

y4 = -3
for x in range(3):
    add_objects(3.25, y4, SHELVES_X, SHELVES_Y)
    add_objects(3.75, y4, SHELVES_X, SHELVES_Y)
    y4 += 4.01

y5 = -3
for x in range(3):
    add_objects(6.75, y5, SHELVES_X, SHELVES_Y)
    add_objects(7.25, y5, SHELVES_X, SHELVES_Y)
    y5 += 4.01


'''
box_center_x, box_center_y = 2.0, 0.0
box_size = 1.7  # meters

# Convert corners to map pixels
x1, y1 = world_to_map(box_center_x - box_size / 2, box_center_y - box_size / 2)
x2, y2 = world_to_map(box_center_x + box_size / 2, box_center_y + box_size / 2)

print("Box 1: ", x1, y1, x2, y2)

og[x1:x2, y1:y2,] = OCCUPIED


box_center_x, box_center_y = 2.0, 1.0
box_size = 1.7  # meters

print("Box 2: ", x1, y1, x2, y2)

# Convert corners to map pixels
x1, y1 = world_to_map(box_center_x - box_size / 2, box_center_y - box_size / 2)
x2, y2 = world_to_map(box_center_x + box_size / 2, box_center_y + box_size / 2)


og[x1:x2, y1:y2,] = OCCUPIED


box_center_x, box_center_y = 4.0, 0.0
box_size = 1.7  # meters

print("Box 3: ", x1, y1, x2, y2)


# Convert corners to map pixels
x1, y1 = world_to_map(box_center_x - box_size / 2, box_center_y - box_size / 2)
x2, y2 = world_to_map(box_center_x + box_size / 2, box_center_y + box_size / 2)

og[x1:x2, y1:y2,] = OCCUPIED

box_center_x, box_center_y = 4.0, 1.0
box_size = 1.7  # meters

print("Box 4: ", x1, y1, x2, y2)


# Convert corners to map pixels
x1, y1 = world_to_map(box_center_x - box_size / 2, box_center_y - box_size / 2)
x2, y2 = world_to_map(box_center_x + box_size / 2, box_center_y + box_size / 2)

og[x1:x2, y1:y2,] = OCCUPIED

'''
