import numpy as np

# Map params
MAP_RES = 0.05    # meters per cell
WORLD_SIZE = 10.0
MAP_SIZE = int(WORLD_SIZE / MAP_RES)
MAP_ORIGIN = MAP_SIZE // 2

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

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

