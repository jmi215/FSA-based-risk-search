from constraint import Constraints, EdgeConstraint, VertexConstraint, Conflict
from state import State, Location
from a_star_risk import AStar_risk
from human_motion_simulator import Simulator
import copy
from math import fabs
from itertools import product
from itertools import combinations
import random
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from numpy import arange
import seaborn


class Environment(object):
    def __init__(self, dimension, agents, humans, obstacles, Num_agents, Num_humans):
        # map dimension, obstacles
        self.dimension = dimension
        self.obstacles = obstacles
        self.obstacles_r = obstacles

        self.Num_agents = Num_agents
        self.Num_humans = Num_humans
        self.goal_set = {}

        self.TH = 50 # time horizon window
        self.Max_step = 100
        

        # reward/penalties
        self.rE = 1 # goal reward
        self.rF = -0.5 # conflict penalty
        self.rs = -0.1 # step penalty
        self.rw = 0
        self.ro = 0

        self.SIM = 2000 # simualation number of human path

        self.conflict_num = 0
        self.agent_goals = []

        self.agents = agents
        self.humans = humans
        self.agent_dict = {}
        self.human_dict = {}
        self.plan = {}
        self.conf_dict = {}
        self.TH_conflict = {}
        self.occupy_vertex = {} # human risk constraints
        self.simulated_path_human = {}

        self.risk = {}
        self.risk_set = {}

        self.step_conflict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}
        self.constraint_human = []

        self.a_star_risk = AStar_risk(self)
        self.sumulator = Simulator(self) # human motion simulator



    def get_neighbors(self, state):
        neighbors = []
    
        #Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors

    def get_legal_neighbors_agent(self, state, agent):
        
        if not self.is_at_goal(state[agent],agent):
            for agent1 in self.agent_dict.keys():
                if agent1 != agent:
                    self.obstacles.append((self.agent_dict[agent1]["goal"].location.x, self.agent_dict[agent1]["goal"].location.y))
            neighbors = self.get_neighbors(state[agent])
            self.obstacles = copy.deepcopy(self.obstacles_r)
        else:
            neighbors=[State(state[agent].time + 1, state[agent].location)]

        legal_neighbors = copy.deepcopy(neighbors)
        for item in neighbors:
            for agent in state:
                if item.__eq__(state[agent]):
                    legal_neighbors.remove(item)

        return legal_neighbors

    def get_legal_neighbors_human(self, state, human):
                
        neighbors = self.get_neighbors(state[human])

        legal_neighbors = copy.deepcopy(neighbors)

        for item in neighbors:
            for human in state:
                if item.__eq__(state[human]):
                    legal_neighbors.remove(item)
                if (item.location.x, item.location.y) in self.agent_goals:
                    legal_neighbors.remove(item)

        return legal_neighbors

        
    def get_first_conflict(self, solution): # given a solution, get the first conflict
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                if "agent" in agent_1 or "agent" in agent_2:
                    state_1 = self.get_state(agent_1, solution, t)
                    state_2 = self.get_state(agent_2, solution, t)

                    if state_1.is_equal_except_time(state_2):
                        result.time = state_1.time
                        result.type = Conflict.VERTEX
                        result.location_1 = state_1.location

                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        
                        return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                if "agent" in agent_1 or "agent" in agent_2:
                    state_1a = self.get_state(agent_1, solution, t)
                    state_1b = self.get_state(agent_1, solution, t+1)

                    state_2a = self.get_state(agent_2, solution, t)
                    state_2b = self.get_state(agent_2, solution, t+1)

                    if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                        result.time = state_1a.time
                        result.type = Conflict.EDGE
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        result.location_1 = state_1a.location
                        result.location_2 = state_1b.location
                        
                        return result                
        return False

    def create_constraints_from_conflict(self, conflict):# given a conflict, create constraints
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint
        
        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)
        
            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            try:
                return solution[agent_name][-1]
            except:
                return {}

    def state_valid(self, state): # if state valid return true else return false
        
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2): # if tansition valid return true else return false
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints


    def admissible_heuristic(self, state, goal):# calcuate heursitic cost, distance |x_1 -x_2|+|y_1 - y_2|
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]

        flag = False
        for goal in goal_state:
            if state.is_equal_except_time(goal):
                flag = True

        return flag

    def all_is_at_goal(self):
        for agent in self.agent_dict.keys():
            if 'agent' in agent:
                if not self.is_at_goal(self.agent_dict[agent]["current"],agent):
                    return False
        return True

    def make_agent_dict(self):

             
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            for goal in agent["goal"]:
                self.agent_goals.append((goal[0], goal[1]))
            last_state = State(0, Location(-2, -2))
            new_state = State(0, Location(-1, -1))            
            self.agent_dict.update({agent['name']:{'start':start_state,'current': start_state, 'last':last_state, 'next':new_state, 'goal':self.agent_goals}})

        for human in self.humans:
            start_state = State(0, Location(human['start'][0], human['start'][1]))
            goal_state = State(0, Location(human['goal'][0], human['goal'][1]))
            last_state = State(0, Location(-2, -2))
            new_state = State(0, Location(-1, -1))

            self.human_dict.update({human['name']:{'start':start_state,'current': start_state, 'last':last_state, 'next':new_state, 'goal':goal_state}})

        for agent in self.agent_dict.keys():
            self.conf_dict.update({agent:[]})   
            


    def human_simulation(self):
        # simulate SIM times for human, generate human risk-constraint and human path plan
        time_step = self.human_dict["human0"]["current"].time
        solution_human = {} # simulated human paths

        local_solution ={}# record the simulated path

        for human in self.human_dict.keys():
            local_solution.update({human:[]})

        for i in range(self.SIM):# do simulations
            path = self.sumulator.simulation(self.Max_step)
            for human in path:
                local_solution[human].append(path[human])

        for human in local_solution:
            solution_human.update({human:local_solution[human]})

        if False:
            print("check simulated human paths")
            i = 0
            for human in solution_human:
                print(human)

                for item in solution_human[human]:
                    print("path",i,"======================")
                    for state in item:
                        print(state)
                    print("=============================")
                    i += 1
            input()

        self.occupy_vertex = {}
        occu = {}
        
        for human in solution_human:

            self.occupy_vertex.update({human:{}})
            occu.update({human:{}})
            statis_data = {} # counting occupied vertex information for human at each time step

            for step in range(self.Max_step+1):
                statis_data.update({str(step):[]})
            
            # statis_data: all occupied vertices by a human at each time step
            for i in range(len(solution_human[human])):
                for step in range(len(solution_human[human][i])):
                    statis_data[str(step)].append(solution_human[human][i][step])
                    self.risk.update({(solution_human[human][i][step].location.x, solution_human[human][i][step].location.y):{}})

            for step in range(self.Max_step+1):
                for item in self.risk:
                    self.risk[item].update({step: 0})


            for step in statis_data:
                tem_data = set(statis_data[step])
                self.occupy_vertex[human].update({str(statis_data[step][0].time):[]})

                for it in tem_data:
                    self.occupy_vertex[human][str(statis_data[step][0].time)].append([it,statis_data[step].count(it)/self.SIM]) #{human:{step:[[vertex, probability], [vertex, probabilty],...]}}
        
        # risk of all humans
        for human in self.occupy_vertex:
            for key in self.occupy_vertex[human]:
                for item in self.occupy_vertex[human][key]:
                    tem_risk = self.risk[(item[0].location.x, item[0].location.y)][item[0].time] + item[1]
                    self.risk[(item[0].location.x, item[0].location.y)].update({item[0].time:tem_risk})
        if False:
            print("check risk")
            for item in self.risk:
                print(item, self.risk[item])
            input()

       
        if False: # draw heatmap with probability of each vertex
            all_occu = copy.deepcopy(self.occupy_vertex)
            for human in self.occupy_vertex:
                for step in self.occupy_vertex[human]:
                    fig = plt.figure()
                    ax = fig.add_subplot(1, 1, 1)
                    ax.xaxis.tick_top()

                    grid = [[0 for x in range(self.dimension[0])] for y in range(self.dimension[1])]
                    for v in self.obstacles:
                        grid[v[0]][v[1]] = 1

                    for i in all_occu[human][step]:
                        grid[i[0].location.x][i[0].location.y] += i[1]

                    seaborn.heatmap(grid, square= True, cbar= False, cmap="Greys", linewidths = 1, linecolor="black", annot= True, fmt=".3g")
                    for i in self.occupy_vertex[human][step]:
                        ax.add_patch(patches.Rectangle((i[0].location.y, i[0].location.x),1,1,edgecolor = 'red',facecolor = 'red',fill=False))

                    ax.set_title('j')
                    ax.set_ylabel('i')
                    fig.savefig("log_path/constraint_"+str(human)+"_TH_"+str(self.TH)+"_step_"+str(i[0].time)+".pdf")
                    plt.clf()
                    plt.close()

    
    def compute_solution(self, alpha, beta): # generate path of each agent with A*, return one plan
        solution = {}

        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            agent_path = self.a_star.search(agent)

            if not agent_path:
                print("A-star: no solution found !")
                return False
            solution.update({agent:agent_path})

        return solution

    def compute_solution_risk(self, state1, state2): # generate path of each agent with A*, return one plan
        solution = {}
        if False:
            print("test in compute_solution_risk")
            print("state1", state1, "state2", state2)
            for agent in state1:
                print(agent, state1[agent], state2[agent])
            input()


        for agent in self.agent_dict.keys():
            input_state = state1[agent]
            g_state = state2[agent]
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            agent_path = self.a_star_risk.search(agent, input_state, g_state)
            if False:
                for item in agent_path:
                    print(item)
                input()

            if not agent_path:
                print("A-star_risk: no solution found !")
                return False
            solution.update({agent:agent_path})

        return solution


    def compute_solution_cost(self, solution):# return the sum of individual length of the path in a solution
        return sum([len(path) for path in solution.values()])

    
    def conflict_dect_vertex(self):# check vertex conflict for next positions s', (s, a → s')
        conflict =set()

        if self.Num_agents >1:
            for agent_1, agent_2 in combinations(self.agent_dict.keys(), 2):
                state_1b = self.agent_dict[agent_1]["next"]
                state_2b = self.agent_dict[agent_2]["next"]

                if state_1b.__eq__(state_2b):# vertex coflict with agents
                    conflict.add(agent_1)
                    conflict.add(agent_2)
                    self.step_conflict[state_1b.time] += 1

        for agent in self.agent_dict.keys():
            for human in self.human_dict.keys():
                if self.agent_dict[agent]["next"].__eq__(self.human_dict[human]["next"]):# vertex coflict with humans
                    conflict.add(agent)
                    self.step_conflict[self.agent_dict[agent]["next"].time] += 1
                
        return conflict
    

    def conflict_dect_edge(self): # check vertex conflict for next positions s', (s, a → s')

        conflict =set()
        if self.Num_agents >1:
            for agent_1, agent_2 in combinations(self.agent_dict.keys(), 2):
                state_1a = self.agent_dict[agent_1]["current"]
                state_1b = self.agent_dict[agent_1]["next"]

                state_2a = self.agent_dict[agent_2]["current"]
                state_2b = self.agent_dict[agent_2]["next"]

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):# edge coflict with agents
                    conflict.add(agent_1)
                    conflict.add(agent_2)
                    self.step_conflict[self.agent_dict[agent_1]["next"].time] += 1


        for agent in self.agent_dict.keys():
            for human in self.human_dict.keys():
                if self.agent_dict[agent]["current"].is_equal_except_time(self.human_dict[human]["next"]) and \
                    self.agent_dict[agent]["next"].is_equal_except_time(self.human_dict[human]["current"]):# vertex coflict with humans
                    conflict.add(agent)
                    self.step_conflict[self.agent_dict[agent]["next"].time] += 1

        return conflict

    def count_conflict_reward(self):

        reward = 0
        for agent in self.plan:
            T = len(self.plan[agent])

        for agent in self.agent_dict.keys():
            self.conf_dict.update({agent:[]}) 

        self.conflict_num = 0

        for t in range(T-1):
            for agent in self.agent_dict.keys():
                self.agent_dict[agent]["current"] = copy.deepcopy(self.plan[agent][t])
                self.agent_dict[agent]["next"] = copy.deepcopy(self.plan[agent][t+1])
            for human in self.human_dict.keys():
                self.human_dict[human]["current"] = copy.deepcopy(self.plan[human][t])
                self.human_dict[human]["next"] = copy.deepcopy(self.plan[human][t+1])

            step_reward = self.get_reward_test()
            reward += step_reward
        arrive_at_goal = 1 if self.conflict_num == 0 else 0

        return reward, arrive_at_goal, self.conflict_num, self.step_conflict


    def get_reward_test(self):

        conflict_v = []
        conflict_e = []
        reward = 0
    
        for agent in self.agent_dict.keys():
            if 'agent'in agent:
                if self.agent_dict[agent]["next"].location != self.plan[agent][-1].location:
                    if self.agent_dict[agent]["next"].location != self.agent_dict[agent]["current"].location:
                        reward += self.rs
                    else:
                        reward += self.rs + self.rw
                else:
                    if self.agent_dict[agent]["current"].location == self.plan[agent][-1].location:
                        reward += 0
                    else:
                        reward += self.rE +self.rs

        conflict_v = self.conflict_dect_vertex()

        if conflict_v:

            for agent in conflict_v:
                if True:
                    reward += self.rF
                    self.conflict_num += 1
                    self.conf_dict[agent].append(1)

        conflict_e = self.conflict_dect_edge()
        if conflict_e:
            for agent in conflict_e:
                if True:
                    reward += self.rF
                    self.conflict_num += 1
                    self.conf_dict[agent].append(1)
          
        # oscilation penalty
        for agent in self.agent_dict.keys():
            if 'agent' in agent:
                if self.agent_dict[agent]["next"].location == self.agent_dict[agent]["last"].location and self.agent_dict[agent]["current"].location != self.agent_dict[agent]["next"].location:
                    reward += self.ro

        return reward



    def get_reward(self):

        conflict_v = []
        conflict_e = []
        reward = 0
    
        for agent in self.agent_dict.keys():
            if 'agent'in agent:
                if not self.is_at_goal(self.agent_dict[agent]["next"], agent):
                    if self.agent_dict[agent]["next"].location != self.agent_dict[agent]["current"].location:
                        reward += self.rs
                    else:
                        reward += self.rs + self.rw
                else:
                    if self.is_at_goal(self.agent_dict[agent]["current"], agent):
                        reward += 0
                    else:
                        reward += self.rE +self.rs

        conflict_v = self.conflict_dect_vertex()

        if conflict_v:

            for agent in conflict_v:
                if True:
                    reward += self.rF
                    self.conflict_num += 1
                    self.conf_dict[agent].append(1)

        conflict_e = self.conflict_dect_edge()
        if conflict_e:
            for agent in conflict_e:
                if True:
                    reward += self.rF
                    self.conflict_num += 1
                    self.conf_dict[agent].append(1)
          
        # oscilation penalty
        for agent in self.agent_dict.keys():
            if 'agent' in agent:
                if self.agent_dict[agent]["next"].location == self.agent_dict[agent]["last"].location and self.agent_dict[agent]["current"].location != self.agent_dict[agent]["next"].location:
                    reward += self.ro

        return reward

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list

        return plan

