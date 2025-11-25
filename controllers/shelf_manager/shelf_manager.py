"""shelf_manager controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor


# Shelf Dimensions
SHELF_WIDTH = 2
SHELF_DEPTH = 0.5
SHELF_SPACING = 0.01

# Aisle Dimensions
ROWS = 14
COLUMNS = 10
AISLE_WIDTH = 2.5

START_X = -11
START_Y = -10

supervisor = Supervisor()

root_node = supervisor.getRoot()
children = root_node.getField("children")
shelves = root_node.

east_facing = True
for row in range(ROWS):
    if row % 2 == 0:
        east_facing = True
    for col in range(COLUMNS):
        current_x = START_X + (row * SHELF_DEPTH) + ((row//2) * AISLE_WIDTH)
        current_y = START_Y + (col * (SHELF_WIDTH + SHELF_SPACING))

        if east_facing:
            shelf = (f'Shelf {{ '
                    f'translation {current_x} {current_y} 0 '
                    f'rotation -0.5773451 0.577351 0.577351 -2.0944 '
                    f'name "shelf_{row}_{col}"'
                    f' }} ')
        else:
            shelf = (f'Shelf {{ '
                    f'translation {current_x} {current_y} 0 '
                    f'name "shelf_{row}_{col}"'
                    f' }} ')

        children.importMFNodeFromString(-1, shelf)
    east_facing = False

print("Shelves placed")