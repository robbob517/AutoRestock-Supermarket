"""supermarket_manager controller."""

from controller import Supervisor
import random
import json

# World Size
WORLD_WIDTH = 20
WORLD_LENGTH = 30

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

# Adjustable
FULLNESS = 0.3 # Chance of product being placed
ROBOT_COUNT = 1 # Number of restockers

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

TIMESTEP = 16

shelf_group = supervisor.getFromDef("SHELVES")
shelf_children = shelf_group.getField("children")

product_group = supervisor.getFromDef("PRODUCTS")
product_children = product_group.getField("children")

robots_group = supervisor.getFromDef("ROBOTS")
robots_children = robots_group.getField("children")

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

    print("Supervisor: Shelves placed and products placed")

def product_placement(shelf_x, shelf_y, shelf_row, shelf_col, east_facing):

    if len(PRODUCTS) > 0:

        shelf_pos = (shelf_row, shelf_col)

        product = (PRODUCTS[shelf_pos])

        item_spacing = 0.3


        product_w = PLACEHOLDER_PRODUCT["w"] # Width of product

        shelf_num = 0
        item_index = 0

        if product["name"] not in shelves: # Initialise dict for current shelf
            shelves[product["name"]] = {"product_type" : PRODUCTS[shelf_pos]["type"],
                                            "empty_positions" : [],
                                            "shelf_grid" : (shelf_row, shelf_col),
                                            "shelf_pos" : (shelf_x, shelf_y)
                                        }

        for z_level in SHELF_LEVELS:
            z_level = round(z_level + 0.1, 2) # Raise product slightly to avoid clipping with shelf when loading

            for i in range(ITEMS_PER_ROW):

                y_offset = round((shelf_y - (SHELF_WIDTH / 2) - SHELF_THICKNESS + ((i + 1) * (product_w + item_spacing))), 2)

                if random.random() > (1 - FULLNESS):  # Likelihood of placing product on shelf
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
                    item_index += 1

                else:   # Add empty item slot to dictionary
                    # Dictionary Structure
                    # Product name : {product_type : { empty_positions : [(x,y,z)], shelf_grid : (row, col), shelf_pos : (shelf_x, shelf_y) } }

                    product_position = (shelf_x, y_offset, z_level)
                    shelves[product["name"]]["empty_positions"].append(product_position)

            shelf_num += 1

def add_robots(robot_count):
    for i in range(robot_count):
        robot = (f'DEF PR2_{i+1} Pr2 {{ '
                 f'translation {-(WORLD_WIDTH/2) + ((i+1)*3)}, -12, 0 '
                 f'rotation 0 0 1 1.5708 '
                 f'name "pr2_{i+1}" '
                 f'controller "marl_controller" '
                 f'supervisor TRUE '
                 f' }} '
                 )

        robots_children.importMFNodeFromString(-1, robot)

def add_product_at_pos(product, position):
    z_level = 0.1

    empty_pos = shelves[product]["empty_positions"]

    for i in range(len(shelves[product]["empty_positions"])):
        if empty_pos[0] == position[0] and empty_pos[1] == position[1]:
            z_level = empty_pos[2]
            empty_pos.remove(i)



    prod = (f'{PLACEHOLDER_PRODUCT["name"]} {{ '
            f'translation {position[0]} {position[1]} {z_level + 0.1} '
            f'name "{product}" '
            f' }} ')

    product_children.importMFNodeFromString(-1, prod)
    print(f"Supervisor: Added product {product} at position: ({position[0]}, {position[1]}, {z_level})")

# Main Loop
def run():

    calculate_shelf_levels()
    shelf_placement()
    add_robots(ROBOT_COUNT)

    # Exporting supermarket data for qlearning
    output_file = "supermarket_data.json"
    print(f"Server: Exporting {len(shelves)} products to {output_file}...")
    with open(output_file, "w") as f:
        json.dump(shelves, f, indent=4)
    print("Server: Export complete.")

    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(TIMESTEP)

    # Current inventory structure, { Item Name : Current stock of item }
    current_inventory = {product["name"] : (ITEMS_PER_SHELF - len(shelves[product["name"]]["empty_positions"])) for
                         product in PRODUCTS.values()}

    # Robot positions structure, { robot_name : { position : pos, orientation : o } }
    robots_positions = {}

    while supervisor.step(TIMESTEP) != -1:

        # Check for messages from robots
        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            data = json.loads(message)

            # Data format, {Type : type, robot_id : id, position : pos, orientation : d}
            if data["type"] == "UPDATE_POS":
                robots_positions[data["robot_id"]] = { "position" : data["position"], "orientation" : data["orientation"] }

            # Data format, {Type : type, robot_id : id, item_name : name, item_position : pos }
            elif data["type"] == "RESTOCKING":
                product = data["item_name"]
                product_position = data["item_pos"]
                current_inventory[product] += 1

                add_product_at_pos(product, product_position)
                print(f"Server: Received inventory for {product} at position {data['item_pos']}")

            receiver.nextPacket()

        # Calculate rewards
        total_possible_stock = len(PRODUCTS) * ITEMS_PER_SHELF
        current_total_stock = sum(current_inventory.values())

        out_of_stock_penalty = 0
        for item, count in current_inventory.items():
            if count == 0:
                out_of_stock_penalty += 50
            elif count <= 5:
                out_of_stock_penalty += 10

        filled_percentage = (current_total_stock / total_possible_stock) * 100

        global_reward = filled_percentage - out_of_stock_penalty

        # Send messages to robots
        global_broadcast = {
            "type" : "GLOBAL",
            "reward" : global_reward,
            "robots_positions" : robots_positions,
            "inventory" : current_inventory,
        }
        emitter.send(json.dumps(global_broadcast))

run()
