import os
import random
import pickle
import json
import time
import numpy as np

class PR2_qlearn_Agent:
    def __init__(self, robot, timestep):

        self.Alpha =0.1
        self.Gamma =0.9
        self.Epsilon =1.0
        self.MIN_Epsilon =0.05
        self.DECAY =0.999
        self.CARGO_location = (0.0, 0.0)
        self.ITEM_properties = self.load_map_data()
        self.robot = robot
        self.TIMESTEP = timestep

        self.ACTIONS = list(self.ITEM_properties.keys())
        self.NUM_ACTIONS = len(self.ACTIONS)
        self.q_table_file = "pr2_q_memory.pkl"
        self.q_table = self.load_q_table()


        self.inventory = {k: 0 for k in self.ACTIONS}

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def load_map_data(self):
        file_path = "supermarket_data.json"
        while not os.path.exists(file_path):
            time.sleep(2)
        with open(file_path,"r") as f:
            raw_data = json.load(f)

        processed_props= {}
        for name, data in raw_data.items():
            if data["empty_positions"]:
                target_xyz = data["empty_positions"][0]
                target_coords= (target_xyz[0], target_xyz[1])
            else:
                target_coords= (0.0, 0.0)

            dist = self.heuristic(self.CARGO_location, target_coords)
            processed_props[name] = {
                "dist":dist,
                "type":data["product_type"],
                "coords":target_coords,
                "start": 20 - len(target_coords)    # Starting stock
            }
        return processed_props


    def load_q_table(self): # if q table already exists use info from q table
            if os.path.exists(self.q_table_file):
                with open(self.q_table_file,'rb') as f:
                    return pickle.load(f)
            else:
                return {}

    def save_q_table(self): # save existing q table
            print("saving q table")
            with open(self.q_table_file,'wb') as f:
                pickle.dump(self.q_table,f)

    # implementing a system where amount of stocked items is categorized into 4 levels so that for larger sets q table doesnt need to grow exponentially
    def get_discrete_level(self, count):
        if count <= 5:
            return 0  #Bad
        elif count <= 14:
            return 1  #OK
        elif count <= 19:
            return 2  #Good
        else:
            return 3 #Stocked (Full)

    def get_state_tuple(self): # gets category of item from the number of items in stock
        current_levels = []
        for item in self.ACTIONS:
            raw_count =self.inventory[item]
            state_cat =self.get_discrete_level(raw_count)
            current_levels.append(state_cat)
        return tuple(current_levels)

    def get_q_values(self, state):
        if state not in self.q_table:
            self.q_table[state] = np.zeros(self.NUM_ACTIONS)
        return self.q_table[state]

    def choose_action(self, state): # decides on the action
        if random.uniform(0, 1) < self.Epsilon:
            return random.randint(0,self.NUM_ACTIONS - 1)
        else:
            q_vals = self.get_q_values(state)
            return np.argmax(q_vals)

    def execute_action_text_mode(self, action_idx, step_num):
        # text mode to check if q table is running well ( not really needed for actual robot)
        item_name= self.ACTIONS[action_idx]
        props= self.ITEM_properties[item_name]
        print(f"   [Step {step_num} | Eps: {self.Epsilon:.4f}] Stocking {item_name} (Type: {props['type']})...")
        if self.inventory[item_name] <20:
            self.inventory[item_name]+= 1
            new_level = self.inventory[item_name]
            status = "CRITICAL" if new_level <= 5 else "OK" if new_level <= 14 else "GOOD" if new_level <= 19 else "FULL"
            print(f"   [RESULT] Level: {new_level}/20 ({status})")
            return True
        else:
            print(f"   [FAIL] {item_name} is already at 20.")
            return False

    def calculate_reward(self, old_inv, action_idx, success):
        item_name = self.ACTIONS[action_idx]
        props = self.ITEM_properties[item_name]
        old_count = old_inv[item_name]
        old_cat = self.get_discrete_level(old_count)

        base_reward = 0

        if old_cat == 0:
            base_reward= 80
        elif old_cat == 1:
            base_reward= 40
        elif old_cat == 2:
            base_reward= 10
        elif old_cat == 3:
            base_reward= -20

        # multiplier is higher if frozen goods are done first , then produce then non food since no real time constraint

        multiplier= 1.0
        if props['type'] == "FROZEN":
            multiplier= 2.0
        elif props['type'] == "PRODUCE":
            multiplier= 1.5
        elif props['type'] == "NON_FOOD":
            multiplier= 0.5

        final_stock_reward = base_reward * multiplier
        distance_penalty = props['dist'] * 0.5
        total_reward = final_stock_reward - distance_penalty

        if not success and old_count != 20:
            total_reward -= 5

        return total_reward

    def update_q_table(self,s,a,r,s_prime):
        q_values = self.get_q_values(s)
        old_q = q_values[a]
        next_max = np.max(self.get_q_values(s_prime))
        new_q = old_q + self.Alpha * (r + self.Gamma * next_max - old_q)
        self.q_table[s][a] = new_q

    def save_restocking_queue(self, action_log):
        filename = "optimal_restocking_path.json"
        print(f"Saving Path: Writing {len(action_log)} steeps to {filename}")
        with open(filename, 'w') as f:
            json.dump(action_log, f, indent=4)


    # Text mode training for testing
    def train_text_mode(self):
        # used to check if the q table is working correctly will be removed when the robot works on its own
        print(f"Starting Q-Learning ")
        episode = 0
        steps_this_day = 0
        current_day_log = []
        while self.robot.step(self.TIMESTEP) != -1:
            episode+=1
            steps_this_day+= 1

            state =self.get_state_tuple()
            old_inv = self.inventory.copy()
            action_idx = self.choose_action(state)
            success =self.execute_action_text_mode(action_idx, steps_this_day)
            if success:
                item_name = self.ACTIONS[action_idx]
                coords = self.ITEM_properties[item_name]["coords"]
                current_day_log.append([list(coords), item_name])

            reward = self.calculate_reward(old_inv, action_idx, success)
            next_state = self.get_state_tuple()
            self.update_q_table(state,action_idx,reward,next_state)

            if self.Epsilon > self.MIN_Epsilon:
                self.Epsilon *= self.DECAY

            if all(value == 20 for value in self.inventory.values()):
                print(f"\n Store fully stocked")

                self.save_q_table()

                if steps_this_day < 350:
                    self.save_restocking_queue(current_day_log)
                    break
                self.inventory = {k: 0 for k in self.ACTIONS}
                steps_this_day = 0
                current_day_log = []