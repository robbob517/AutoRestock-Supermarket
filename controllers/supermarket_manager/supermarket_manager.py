"""supermarket_manager controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import random


# Shelf Dimensions
SHELF_HEIGHT = 2.5
SHELF_WIDTH = 4
SHELF_DEPTH = 0.5
SHELF_SPACING = 0.01
SHELF_COUNT = 4
SHELF_THICKNESS = 0.05

# Aisle Dimensions
ROWS = 10
COLUMNS = 4
AISLE_WIDTH = 2.5

START_X = -7.25
START_Y = -3

PRODUCTS = [
    {
        "name": "BiscuitBox",
        "w": 0.24,
        "h": 0.04,
        "d": 0.08
    },

    {
        "name": "CerealBox",
        "w": 0.08,
        "h": 0.3,
        "d": 0.2
    },

    {
        "name": "HoneyJar",
        "h": 0.115,
        "r": 0.045
    },

    {
        "name": "Can",
        "h": 0.1222,
        "r": 0.03175
    }
            ]

SHELF_LEVELS = []

supervisor = Supervisor()

shelf_group = supervisor.getFromDef("SHELVES")
shelf_children = shelf_group.getField("children")

product_group = supervisor.getFromDef("PRODUCTS")
product_children = product_group.getField("children")

shelf_spacing = (SHELF_HEIGHT - (SHELF_COUNT * SHELF_THICKNESS)) / SHELF_COUNT

def calculate_shelf_levels():

    for i in range(SHELF_COUNT):
        SHELF_LEVELS.append( SHELF_THICKNESS + i * (shelf_spacing + SHELF_THICKNESS/2) )

def shelf_placement():

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
                        f'rotation 0.5773451 0.577351 0.577351 2.0944 '
                        f'name "shelf_{row}_{col}"'
                        f' }} ')

            shelf_children.importMFNodeFromString(-1, shelf)

            product_placement(current_x, current_y, row, col, east_facing)

        east_facing = False

    print("Shelves placed")

def product_placement(shelf_x, shelf_y, shelf_col, shelf_row, east_facing):

    product = random.choice(PRODUCTS)
    available_space = SHELF_WIDTH - (2 * SHELF_THICKNESS)
    item_spacing = 0.3
    fullness = 0.5 # Chance of product being placed


    if product["name"] == "CerealBox" or product["name"] == "BiscuitBox":
        product_w = (product["w"])

    else:
        product_w = (product["r"] * 2)

    items_per_row = (int)(available_space // (product_w + item_spacing))
    for z_level in SHELF_LEVELS:
            z_level += 0.1
            for i in range(items_per_row):

                if random.random() > fullness:
                    y_offset = ( shelf_y - (SHELF_WIDTH/2) - SHELF_THICKNESS + ((i+1)* (product_w + item_spacing) ))

                    if east_facing:
                        prod = (f'{product["name"]} {{ '
                                f'translation {shelf_x} {y_offset} {z_level} '
                                f'rotation 0 0 1 3.14159 '
                                f'name "product_{shelf_row}_{shelf_col}_{i}" '
                                f' }} ')
                    else:
                        prod = (f'{product["name"]} {{ '
                                f'translation {shelf_x} {y_offset} {z_level} '
                                f'name "product_{shelf_row}_{shelf_col}_{i}" '
                                f' }} ')

                    product_children.importMFNodeFromString(-1, prod)


# Main
calculate_shelf_levels()
shelf_placement()