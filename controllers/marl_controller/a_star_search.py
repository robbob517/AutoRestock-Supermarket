import heapq
import math
import supermarket_map


# Function to calculate Manhattan Distance
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

# Function to find free neighbors of current node
def free_neighbors(node):
    x, y = node
    steps = [(-1,0),(1,0),(0,-1),(0,1)]  # Up, down, left, right
    result = []
    for dx, dy in steps:
        nx, ny = x+dx, y+dy
        if 0 <= nx < supermarket_map.MAP_SIZE and 0 <= ny < supermarket_map.MAP_SIZE and supermarket_map.og[nx][ny]==1:
            result.append((nx, ny))
    return result

# Function to calculate F-Cost of nodes
def calc_f_cost(current_node, goal, g_cost):
    h_cost = heuristic(current_node, goal)
    return g_cost + h_cost

def get_nearest_walkable_node(target_node):#
    target_x, target_y = target_node

    if 0 <= target_x < supermarket_map.MAP_SIZE and 0 <= target_y < supermarket_map.MAP_SIZE:
        if supermarket_map.og[target_x][target_y] == 1:
            return target_node

    queue = [target_node]
    visited = {target_node}

    while queue:
        current_x, current_y = queue.pop(0)
        if 0 <= current_x < supermarket_map.MAP_SIZE and 0 <= current_y < supermarket_map.MAP_SIZE:
            if supermarket_map.og[current_x][current_y] == 1:
                return (current_x, current_y)

        steps = [(-1,0),(1,0),(0,-1),(0,1)]
        for dx, dy in steps:
            nx, ny = current_x + dx, current_y + dy
            if (nx, ny) not in visited:
                if abs(nx - target_x) + abs(ny - target_y) < 20:
                    queue.append((nx, ny))
                    visited.add((nx, ny))

    return None

# Main function to find and construct a map from start node to goal node
def a_star_path(start, goal):
    raw_start_node = supermarket_map.world_to_map(start[0], start[1])
    raw_goal_node = supermarket_map.world_to_map(goal[0], goal[1])

    start_node = get_nearest_walkable_node(raw_start_node)

    goal_node = get_nearest_walkable_node(raw_goal_node)
    goal = supermarket_map.map_to_world(goal_node[0], goal_node[1])

    # print(f"\nStart Node: {start_node}")
    # print(f"Old goal: {raw_goal_node}, New goal: {goal_node}\n")

    if goal_node is None:
        return None

    open_set = []
    came_from = {}
    g_cost = {start_node: 0}
    closed_set = set()

    # Priority Queue stores: (F-Score, Current_Node)
    heapq.heappush(open_set, (0, start_node))

    while open_set:
        # Check if active node is goal node and construct the whole path to return
        _, active_node = heapq.heappop(open_set)
        if active_node == goal_node:
            path = []
            while active_node in came_from:
                path.append(active_node)
                active_node = came_from[active_node]
            path.append(start_node)
            path.reverse()
            return path # Exit main while loop

        closed_set.add(active_node)

        # Add current node neighbours to priority queue
        for neighbor in free_neighbors(active_node):
            if neighbor not in closed_set:
                neighbor_g_cost = g_cost[active_node] + 1
                # Only add neighbor if it's a new node in queue or if it's a shortest path then before
                if neighbor not in g_cost or neighbor_g_cost < g_cost[neighbor]:
                    came_from[neighbor] = active_node
                    g_cost[neighbor] = neighbor_g_cost
                    neighbor_f_cost = calc_f_cost(neighbor, goal, neighbor_g_cost)
                    heapq.heappush(open_set, (neighbor_f_cost, neighbor))

    print("No path found!")
    return None

# Function to calculate turning angle for robot to follow the path
def get_rotation(current, target):
    directions = ['up', 'right', 'down', 'left']  # clockwise
    current_idx = directions.index(current)
    target_idx = directions.index(target)
    diff = target_idx - current_idx
    # Choose the shortest rotation
    if diff == 1 or diff == -3:
        return -math.pi/2    # 90 degrees right
    elif diff == -1 or diff == 3:
        return math.pi/2   # 90 degrees left
    elif diff == 2 or diff == -2:
        return math.pi      # 180 degrees
    else:
        return 0            # already facing

# Function to get instructions according to calculated path for robot to follow
def move_instructions(world_path):
    instructions = []
    for i in range(1, len(world_path)):
        x_prev, y_prev = world_path[i - 1]
        x_next, y_next = world_path[i]

        if x_next > x_prev:
            target_dir = 'right'
        elif x_next < x_prev:
            target_dir = 'left'
        elif y_next > y_prev:
            target_dir = 'up'
        elif y_next < y_prev:
            target_dir = 'down'

        instructions.append((target_dir, x_next, y_next))
    return instructions

def smooth_path(path):
    if not path or len(path) < 3:
        return path

    simplified_path = [path[0]]
    direction_old = (
        path[1][0] - path[0][0],
        path[1][1] - path[0][1]
    )

    for i in range(2, len(path)):
        direction_new = (
            path[i][0] - path[i - 1][0],
            path[i][1] - path[i - 1][1]
        )

        if direction_new != direction_old:
            simplified_path.append(path[i-1])
            direction_old = direction_new

    simplified_path.append(path[-1])
    return simplified_path


