import numpy as np

# Shelf Dimensions
SHELF_WIDTH = 4
SHELF_DEPTH = 0.5
SHELF_SPACING = 0.01

# Aisle Dimensions
ROWS = 10
COLUMNS = 4
AISLE_WIDTH = 2.5

START_X = -7.25
START_Y = -3

MAP_RES = 0.2
WORLD_SIZE = 30
MAP_SIZE = int(WORLD_SIZE / MAP_RES)
MAP_ORIGIN = MAP_SIZE // 2

FREE = 1
OCCUPIED = 0

og = np.full((MAP_SIZE, MAP_SIZE), FREE, dtype=int)

def world_to_map(wx, wy):
    mx = int(round(wx / MAP_RES)) + int(MAP_ORIGIN)
    my = int(round(wy / MAP_RES)) + int(MAP_ORIGIN)

    mx = max(0, min(mx, MAP_SIZE - 1))
    my = max(0, min(my, MAP_SIZE - 1))

    return mx, my

def map_to_world(mx, my):
    wx = (mx - MAP_ORIGIN) * MAP_RES
    wy = (my - MAP_ORIGIN) * MAP_RES
    return wx, wy

def update_object(center_x, center_y, size_x, size_y, margin = 0.6, occupied=True):

    start_x_m = center_x - (size_x / 2) - margin
    end_x_m = center_x + (size_x / 2) + margin

    start_y_m = center_y - (size_y / 2) - margin
    end_y_m = center_y + (size_y / 2) + margin

    x1, y1 = world_to_map(start_x_m, start_y_m)
    x2, y2 = world_to_map(end_x_m, end_y_m)

    if occupied:
        og[x1:x2, y1:y2] = OCCUPIED
    else:
        og[x1:x2, y1:y2] = FREE

# Bounding Walls
update_object(-10, 0, 0.2, 30) # East Wall
update_object(10, 0, 0.2, 30) # West Wall
update_object(0, 14.9, 20, 0.2) # North Wall
update_object(0, -14.9, 20, 0.2) # South Wall
update_object(-5.65, -8, 8.5, 0.2) # Storage Divider Right
update_object(5.65, -8, 8.5, 0.2) # Storage Divider Left

for row in range(ROWS):
    for col in range(COLUMNS):
        current_x = START_X + (row * SHELF_DEPTH) + ((row // 2) * AISLE_WIDTH)
        current_y = START_Y + (col * (SHELF_WIDTH + SHELF_SPACING))

        update_object(current_x, current_y, SHELF_DEPTH, SHELF_WIDTH, 0.8)


