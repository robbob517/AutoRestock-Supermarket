import pr2_controller as pr2
import pr2_qlearn_agent as qlearn
import odometry as od
import a_star_search as pathfind

TIMESTEP = 16

STATE_IDLE = 0
STATE_PLANNING = 1
STATE_MOVING = 2
STATE_REFILLING = 3
STATE_RESTOCKING = 4

def run():
    robot = pr2.robot
    robot_name = robot.getName()
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(TIMESTEP)

    pr2.initialize_devices()
    pr2.enable_devices()
    pr2.set_initial_position()

    robot_state = STATE_IDLE



    while robot.step(TIMESTEP) != -1:

        # Update required variables per timestep


        # Marl Loop
        # Choose a new action
        if robot_state == STATE_IDLE:
            pass

        # Calculating path to goal position
        elif robot_state == STATE_PLANNING:
            pass

        # Move to goal position
        elif robot_state == STATE_MOVING:
            pass

        # Return to storage area to refill local inventory
        elif robot_state == STATE_REFILLING:
            pass

        # Send restocking instruction to supermarket manager
        elif robot_state == STATE_RESTOCKING:
            pass

run()

