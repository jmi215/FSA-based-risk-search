import time
import copy
from state import State, Location
from math import fabs

class AStar_risk():
    def __init__(self, env):
        self.env = env
        self.agent_dict = env.agent_dict
        #self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        self.alpha = 0.01
        self.beta = 0.01
        self.gamma = 0.1

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)

        return total_path[::-1]## [::-1]copy from last one to first one

    def admissible_heuristic(self, state, goal):# calcuate heursitic cost, distance |x_1 -x_2|+|y_1 - y_2|

        return fabs(state.location.x - goal[0]) + fabs(state.location.y - goal[1])

    def get_risk(self, state):

        try:
            risk = self.env.risk[(state.location.x, state.location.y)][state.time]
        except:
            risk = 0

        return risk

    def search(self, agent_name, state_1, state_2):

        initial_state = state_1

        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}
        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 
        f_score[initial_state] = self.admissible_heuristic(initial_state, state_2)
        

        while open_set:

            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)##return minum fscore value 

            if (current.location.x, current.location.y)==state_2:
                return self.reconstruct_path(came_from, current)

            open_set -= {current}#remove current state from open set
            closed_set |= {current}#add current state to close set

            neighbor_list = self.get_neighbors(current)

            #remove other goal
            tem_goal_list = copy.deepcopy(self.env.agent_goals)
            tem_goal_list.remove(state_2)
            for item in neighbor_list:
                if (item.location.x, item.location.y) in tem_goal_list:
                    neighbor_list.remove(item)

            current_next_step = State(current.time+1, current.location )


            current_next_risk = self.get_risk(current_next_step)

            for neighbor in neighbor_list:
                if neighbor in closed_set:# if ture, goto next loop
                    continue

                neighbour_current_risk = 0

                neighbour_risk = self.get_risk(neighbor)

                try:
                    neighbour_current_risk += self.env.risk[(neighbor.location.x, neighbor.location.y)][current.time] 
                except:
                    1 

                next_neighbours = self.get_neighbors(neighbor)
                #print("next_neighbours", next_neighbours)
                future_risk = 0
                for state in next_neighbours:
                    future_risk += self.get_risk(state)

                risk = 1/self.alpha * (neighbour_risk + neighbour_current_risk*current_next_risk) + 1/self.beta * future_risk


                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost
                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue
                
                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, state_2) + risk


        return False


