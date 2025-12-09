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

ITEMS_PER_ROW = 5
ITEMS_PER_SHELF = ITEMS_PER_ROW * SHELF_COUNT

PLACEHOLDER_PRODUCT = {
        "name": "CerealBox",
        "w": 0.08,
        "h": 0.3,
        "d": 0.2
    }

PRODUCTS = {
     (0, 0) : {"name" : "BREAD","type" : "DRY"},
     (0, 1) : {"name" : "CEREAL", "type" : "DRY"},
     (0, 2) : {"name" : "WATER","type" : "DRY"},
     (0, 3) : {"name" : "CHIPS","type" : "DRY"},
     (1, 0) : {"name" : "PASTA","type" : "DRY"},
     (1, 1) : {"name" : "KIDNEY BEANS","type" : "DRY"},
     (1, 2) : {"name" : "CHICKPEAS","type" : "DRY"},
     (1, 3) : {"name" : "BAKED BEANS","type" : "DRY"},
     (2, 0) : {"name" : "SOY BEANS","type" : "DRY"},
     (2, 1) : {"name" : "BLACK BEANS","type" : "DRY"},

     (2, 2) : {"name" : "MILK","type" : "PRODUCE"},
     (2, 3) : {"name" : "APPLES","type" : "PRODUCE"},
     (3, 0) : {"name" : "BANANAS","type" : "PRODUCE"},
     (3, 1) : {"name" : "EGGS","type" : "PRODUCE"},
     (3, 2) : {"name" : "YOGURT","type" : "PRODUCE"},
     (3, 3) : {"name" : "KIWI","type" : "PRODUCE"},
     (4, 0) : {"name" : "STRAWBERRY","type" : "PRODUCE"},
     (4, 1) : {"name" : "MANGO","type" : "PRODUCE"},
     (4, 2) : {"name" : "BLUEBERRY","type" : "PRODUCE"},
     (4, 3) : {"name" : "CREAM","type" : "PRODUCE"},

     (5, 0) : {"name" : "ICE_CREAM","type" : "FROZEN"},
     (5, 1) : {"name" : "PIZZA","type" : "FROZEN"},
     (5, 2) : {"name" :  "CHICKEN","type" : "FROZEN"},
     (5, 3) : {"name" : "PEAS","type" : "FROZEN"},
     (6, 0) : {"name" : "ICE LOLLY","type" : "FROZEN"},
     (6, 1) : {"name" : "HASH BROWN","type" : "FROZEN"},
     (6, 2) : {"name" :  "TURKEY","type" : "FROZEN"},
     (6, 3) : {"name" : "BURGER","type" : "FROZEN"},
     (7, 0) : {"name" : "MIXED FRUIT","type" : "FROZEN"},

     (7, 1) : {"name" : "BATH TOWEL","type" : "NON_FOOD"},
     (7, 2) : {"name" : "FACE TOWEL","type" : "NON_FOOD"},
     (7, 3) : {"name" : "HAIR TOWEL","type" : "NON_FOOD"},
     (8, 0) : {"name" : "HAND TOWEL","type" : "NON_FOOD"},
     (8, 1) : {"name" : "SHAMPOO","type" : "NON_FOOD"},
     (8, 2) : {"name" : "CONDITIONER","type" : "NON_FOOD"},
     (8, 3) : {"name" : "BODY WASH","type" : "NON_FOOD"},
     (9, 0) : {"name" :  "TOOTHBRUSH","type" : "NON_FOOD"},
     (9, 1) : {"name" : "TOOTHPASTE","type" : "NON_FOOD"},
     (9, 2) : {"name" : "TOILET ROLL","type" : "NON_FOOD"},
     (9, 3) : {"name" : "SOAP","type" : "NON_FOOD"},
}

SHELF_LEVELS = []

shelves = {}

supervisor = Supervisor()

TIMESTEP = int(supervisor.getBasicTimeStep())

shelf_group = supervisor.getFromDef("SHELVES")
shelf_children = shelf_group.getField("children")

product_group = supervisor.getFromDef("PRODUCTS")
product_children = product_group.getField("children")

def calculate_shelf_levels():
    shelf_spacing = (SHELF_HEIGHT - (SHELF_COUNT * SHELF_THICKNESS)) / SHELF_COUNT

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
                rotation = "-0.577 0.577 0.577 -2.09"
            else:
                rotation = "0.577 0.577 0.577 2.09"

            shelf = (f'Shelf {{ '
                    f'translation {current_x} {current_y} 0 '
                    f'rotation {rotation} '
                    f'name "shelf_{row}_{col}"'
                    f' }} ')

            shelf_children.importMFNodeFromString(-1, shelf)

            product_placement(current_x, current_y, row, col, east_facing)

        east_facing = False

    print("Shelves placed and products placed")

def product_placement(shelf_x, shelf_y, shelf_row, shelf_col, east_facing):

    if len(PRODUCTS) > 0:

        shelf_pos = (shelf_row, shelf_col)

        product = (PRODUCTS[shelf_pos])

        item_spacing = 0.3
        fullness = 0.2 # Chance of product being placed

        product_w = PLACEHOLDER_PRODUCT["w"] # Width of product

        shelf_num = 0

        if product["name"] not in shelves: # Initialise dict for current shelf
            shelves[product["name"]] = {"product_type" : PRODUCTS[shelf_pos]["type"],
                                            "empty_positions" : [],
                                            "shelf_grid" : (shelf_row, shelf_col),
                                            "shelf_pos" : (shelf_x, shelf_y)
                                        }

        for z_level in SHELF_LEVELS:
            item_index = 0
            shelf_num %= SHELF_COUNT
            z_level += 0.1 # Raise product slightly to avoid clipping with shelf when loading

            for i in range(ITEMS_PER_ROW):

                y_offset = (shelf_y - (SHELF_WIDTH / 2) - SHELF_THICKNESS + ((i + 1) * (product_w + item_spacing)))

                if random.random() > (1 - fullness):  # Likelihood of placing product on shelf
                    if east_facing:
                        rotation = "0 0 1 3.14159"
                    else:
                        rotation = "0 0 1 0"

                    prod = (f'{PLACEHOLDER_PRODUCT["name"]} {{ '    # Use placeholder item as product model
                            f'translation {shelf_x} {y_offset} {z_level} '
                            f'rotation {rotation} '
                            f'name "{product["name"]}_{shelf_row}_{shelf_col}_{shelf_num}_{item_index}" '
                            f' }} ')

                    product_children.importMFNodeFromString(-1, prod)

                else:   # Add empty item slot to dictionary
                    # Dictionary Structure
                    # Product name : {product_type : "type", empty_positions : [(x,y,z)], shelf_grid : (row, col), shelf_pos : (shelf_x, shelf_y)}

                    product_position = (shelf_x, y_offset, z_level)
                    shelves[product["name"]]["empty_positions"].append(product_position)

                item_index += 1
            shelf_num += 1

# Main Loop
def run():
    calculate_shelf_levels()
    shelf_placement()

    emitter = supervisor.getEmitter("emitter")
    receiver = supervisor.getReceiver("receiver")
    receiver.enable(TIMESTEP)

    current_inventory = {product["name"]: (ITEMS_PER_SHELF - len(shelves[product["name"]]["empty_positions"])) for
                         product in PRODUCTS.values()}

    while supervisor.step(16) != 1:
        while receiver.getQueueLength() > 0:
            message = receiver.getData().decode('utf-8')
            data = json.loads(message)
            receiver.nextPacket()

            if data["type"] == "RESTOCK":
                item = data["item"]
                current_inventory[item] += 1
                print(f"Robot restocked item {item}. New stock count {current_inventory[item]}")

        total_stock = sum(current_inventory.values())
        global_reward = total_stock

        state_msg = {
            "inventory" : current_inventory,
            "reward" : global_reward,
        }
        emitter.send(json.dumps(state_msg)).encode("utf-8")

output_file = "supermarket_data.json"
print(f"Exporting {len(empty_slots)} products to {output_file}...")
with open(output_file, "w") as f:
    json.dump(empty_slots, f, indent=4)

print("Export complete.")

run()

# Checking number of empty slots
# for x, y in empty_slots.items():
#     print(f"Shelf: {x}, Product Type: {y["product_type"]}, Empty Slots: {len(y["empty_positions"])}, Shelf Grid Pos: {y["shelf_pos"]}")
