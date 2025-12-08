"""supermarket_manager controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import random
import json

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

PLACEHOLDER_PRODUCT = {
        "name": "CerealBox",
        "w": 0.08,
        "h": 0.3,
        "d": 0.2
    }

PRODUCTS = {
        "BREAD" : {"type" : "DRY"},
        "CEREAL" : {"type" : "DRY"},
        "WATER" : {"type" : "DRY"},
        "CHIPS" : {"type" : "DRY"},
        "PASTA" : {"type" : "DRY"},
        "KIDNEY BEANS" : {"type" : "DRY"},
        "CHICKPEAS" : {"type" : "DRY"},
        "BAKED BEANS" : {"type" : "DRY"},
        "SOY BEANS" : {"type" : "DRY"},
        "BLACK BEANS" : {"type" : "DRY"},

        "MILK" : {"type" : "PRODUCE"},
        "APPLES" : {"type" : "PRODUCE"},
        "BANANAS" : {"type" : "PRODUCE"},
        "EGGS" : {"type" : "PRODUCE"},
        "YOGURT" : {"type" : "PRODUCE"},
        "KIWI" : {"type" : "PRODUCE"},
        "STRAWBERRY" : {"type" : "PRODUCE"},
        "MANGO" : {"type" : "PRODUCE"},
        "BLUEBERRY" : {"type" : "PRODUCE"},
        "CREAM" : {"type" : "PRODUCE"},

        "ICE_CREAM" : {"type" : "FROZEN"},
        "PIZZA" : {"type" : "FROZEN"},
        "CHICKEN" : {"type" : "FROZEN"},
        "PEAS" : {"type" : "FROZEN"},
        "ICE LOLLY" : {"type" : "FROZEN"},
        "HASH BROWN" : {"type" : "FROZEN"},
        "TURKEY" : {"type" : "FROZEN"},
        "BURGER" : {"type" : "FROZEN"},
        "MIXED FRUIT" : {"type" : "FROZEN"},

        "BATH TOWEL" : {"type" : "NON_FOOD"},
        "FACE TOWEL" : {"type" : "NON_FOOD"},
        "HAIR TOWEL" : {"type" : "NON_FOOD"},
        "HAND TOWEL" : {"type" : "NON_FOOD"},
        "SHAMPOO" : {"type" : "NON_FOOD"},
        "CONDITIONER" : {"type" : "NON_FOOD"},
        "BODY WASH" : {"type" : "NON_FOOD"},
        "TOOTHBRUSH" : {"type" : "NON_FOOD"},
        "TOOTHPASTE" : {"type" : "NON_FOOD"},
        "TOILET ROLL" : {"type" : "NON_FOOD"},
        "SOAP" : {"type" : "NON_FOOD"},
}

SHELF_LEVELS = []

empty_slots = {}

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

    print("Shelves placed and products placed")

def product_placement(shelf_x, shelf_y, shelf_col, shelf_row, east_facing):

    if len(PRODUCTS) > 0:
        product = random.choice((list)(PRODUCTS.keys()))

        item_spacing = 0.3
        fullness = 0.2 # Chance of product being placed

        product_w = PLACEHOLDER_PRODUCT["w"] # Width of product

        items_per_row = 5
        shelf_num = 0

        if empty_slots.get((shelf_row, shelf_col)) is None: # Initialise dict for current shelf
            empty_slots[product] = {"product_type" : PRODUCTS[product]["type"], "empty_positions" : [], "shelf_pos" : (shelf_row, shelf_col)}

        for z_level in SHELF_LEVELS:
            item_index = 0
            shelf_num %= SHELF_COUNT

            z_level += 0.1 # Raise product slightly to avoid clipping with shelf when loading

            for i in range(items_per_row):

                y_offset = (shelf_y - (SHELF_WIDTH / 2) - SHELF_THICKNESS + ((i + 1) * (product_w + item_spacing)))

                if random.random() > (1 - fullness):  # Likelihood of placing product on shelf
                    if east_facing:
                        prod = (f'{PLACEHOLDER_PRODUCT["name"]} {{ '    # Use placeholder item as product model
                                f'translation {shelf_x} {y_offset} {z_level} '
                                f'rotation 0 0 1 3.14159 '
                                f'name "{product}_{shelf_row}_{shelf_col}_{shelf_num}_{item_index}" '
                                f' }} ')
                    else:
                        prod = (f'{PLACEHOLDER_PRODUCT["name"]} {{ '
                                f'translation {shelf_x} {y_offset} {z_level} '
                                f'name "{product}_{shelf_row}_{shelf_col}_{shelf_num}_{item_index}" '
                                f' }} ')

                    product_children.importMFNodeFromString(-1, prod)

                else:   # Add empty item slot to dictionary
                    # Dictionary Structure
                    # Product name : {product_type : "type", empty_positions : [(x,y,z)], shelf_position : (row, col)}

                    product_position = (shelf_x, y_offset, z_level)
                    empty_slots[product]["empty_positions"].append(product_position)

                item_index += 1

            shelf_num += 1

        # Remove product entry so there are no repeats
        del PRODUCTS[product]

# Main
calculate_shelf_levels()
shelf_placement()

output_file = "supermarket_data.json"
print(f"Exporting {len(empty_slots)} products to {output_file}...")
with open(output_file, "w") as f:
    json.dump(empty_slots, f, indent=4)

print("Export complete.")

# Checking number of empty slots
# for x, y in empty_slots.items():
#     print(f"Shelf: {x}, Product Type: {y["product_type"]}, Empty Slots: {len(y["empty_positions"])}, Shelf Grid Pos: {y["shelf_pos"]}")