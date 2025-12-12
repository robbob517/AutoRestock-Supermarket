# AutoRestock-Supermarket
Automated supermarket restocking system utilising multi-agent reinforcement learning in Webots

Webots Simulator, Python

## Project Overview
The Automation of services in warehouses has become
one of the optimum use cases of robots for labour.
With this project, we are trying to bring this advancement
into the retail sector. Supermarkets are a complex environ-
ment involving tight spaces. We have chosen this problem
as supermarket inventory management is generally a labour-
intensive job prone to human error. With our project, we aim
to alleviate the load of human workers with the development
of an autonomous restocking system using the PR2 robot.
The robot will be capable of making rapid decisions and
navigating of the complex environment. The aim is to create
an autonomous system that can detect stock depletion based
on distance, urgency, and cost to transfer

The following details the implemented functions of the PR2 robot

The robot operates in a closed loop:
1.  the robot gains the information of the stock of the virtual supermarket through the supermarket manager
2.  It then chooses the most optimal way to stock the items using the q learning table
3.  Using odometry it navigates to the coordinates of the item that needs to be stocked

## Key Features
* **Dynamic Environment:** A "Supermarket Manager" controller that generates the shelf layouts and the location of items on the shelf, it then exports map data to JSON.
* **Q-Learning Agent:** A custom reinforcement learning implementation capable of managing 15 distinct items ( each with a stock of 20 ) with a state space optimized via binning.
* **Memory Persistence:** The agent saves its learned Q-Table (`.pkl`) and the optimal restocking path (`.json`) for use.
* **Odometry Navigation:** Main algorithm that allows the PR2 robot to navigate the space.

## File Structure

### `controllers/`
* **`pr2_controller.py`** 
    * Contains the `PR2_qlearn_Agent` class.
    * Handles state discretization, reward calculation, and Q-table updates.
    * Loads map data from `supermarket_data.json`.
* **`supermarket_manager.py`** 
    * Generates the random shelf layout.
    * Calculates item coordinates and exports `supermarket_data.json`.
    * Manages the virtual inventory logic.

### `data/` (Generated at Runtime)
* `supermarket_data.json`: The live map of where items are located.
* `pr2_q_memory.pkl`: The serialized Q-Table which the robot uses as its long term memory.
* `optimal_restocking_path.json`: The final sequence of actions generated after convergence.

## How to Run
1.  Open the world file `supermarket.wbt` in **Webots**.
2.  Ensure the controllers are assigned:
    * **Robot:** `pr2_controller`
    * **Environment:** `supermarket_manager`
3.  Run the simulation.
    * The robot will initially explore (High Epsilon).
    * Watch the console for "Episode" and "Reward" logs.
    * Once the store is fully stocked efficiently (<350 steps), the system saves the optimal path and stops.

## Authors
* **Rayyan Syed:** Q-Learning Logic 
* **Robert Choi:** Environment Design & Multi-Agent Coordination
* **Ahmed Alam:** Odometry 