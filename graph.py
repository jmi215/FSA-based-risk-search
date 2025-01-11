from random import choice
from queue import Queue
import sys
import copy
import math
from math import sqrt, log
import time
from state import State, Location
from constraint import Constraints, EdgeConstraint, VertexConstraint
from itertools import product,combinations
import os
import random
 
class Node(object):
    def __init__(self):
        self.parent = None
        self.children = {}
 
        self.q = 3
        self.value = 0
        self.path = []
        self.state = {}

    def value(self):
        return self.value

    def add_children(self, children):
        for child in children:
            self.children[str(child.q)] = child 

    def __repr__(self):
        return "Node: {}, state: {}, q: {}, v: {}, path:{}".format(
        hash(self), self.state, self.q, self.value, self.path)
 
class Graph(object):
    def __init__(self, int_state_agent, int_state_human, environment):
        self.env = environment

        self.root = Node()
        self.root.state = int_state_agent
        self.root.state_human = int_state_human

        self.q_accept = 0
        self.accept_node = Node()
        self.accept_node.q = self.q_accept
        #self.accept_node.value = 1



    def form_solution(self,node):## to be modified
        tem = {}
        for agent in node.state:
            tem.update({agent:[]})

        if node.path:
            for agent in node.path:
                for item in node.path[agent]:
                    tem[agent].append(item)

        return tem

    def selection(self,node):## select a child with minimum value v indicating safety
        
        ##given a node, return a best child node
        min_value = min(node.children.values(), key=lambda n: n.value()).value()
        min_nodes = [n for n in node.children.values() if n.value() == min_value]
        node = choice(min_nodes)
        
        return node

    def get_children(self,q):
        child_list = []
        if q == 3:
            child_list = [2,6,7]
        elif q == 2:
            child_list = [1,5]
        elif q == 6:
            child_list = [1,4]
        elif q == 7:
            child_list = [5,4]
        elif q == 1 or q == 5 or q == 4:
            child_list = [0]
        else:
            return False
        
        return child_list

    def map_q_state(self,q1,q2):
        for agent in self.env.agent_dict.keys():
            if q1 == 3:
                if q2 == 2:
                    return self.env.agent_dict[agent]["goal"][0]
                elif q2 == 6:
                    return self.env.agent_dict[agent]["goal"][1]
                elif q2 == 7:
                    return self.env.agent_dict[agent]["goal"][2]

            elif q1 == 2:
                if q2 == 1:
                    return self.env.agent_dict[agent]["goal"][1]
                elif q2 == 5:
                    return self.env.agent_dict[agent]["goal"][2]
            elif q1 == 6:
                if q2 == 1:
                    return self.env.agent_dict[agent]["goal"][0]
                elif q2 == 4:
                    return self.env.agent_dict[agent]["goal"][2]
            elif q1 == 7:
                if q2 == 5:
                    return self.env.agent_dict[agent]["goal"][0]
                elif q2 == 4:
                    return self.env.agent_dict[agent]["goal"][1]
            elif q1 == 1:
                if q2 == 0:
                    return self.env.agent_dict[agent]["goal"][2]
            elif q1 == 5:
                if q2 == 0:
                    return self.env.agent_dict[agent]["goal"][1]
            elif q1 == 4:
                if q2 == 0:
                    return self.env.agent_dict[agent]["goal"][0]
            else:
                print("error in map_q_state!")
                return False

    def get_risk(self, state):

        try:
            risk = self.env.risk[(state.location.x, state.location.y)][state.time]
        except:
            risk = 0

        return risk
    def calculate_safety_value(self,solution):
        safety_value = {}
        safety = 0
        for agent in solution:
            base_time = solution[agent][0].time
            #print("base_time",base_time)
            #input()
            safety_value.update({agent:0})
            for item in solution[agent]:
                try:
                    safety_value[agent] += self.env.risk[(item.location.x, item.location.y)][item.time]
                except:
                    safety_value[agent] += 0

                #next_neighbours = self.env.get_neighbors(item)
                #print("next_neighbours", next_neighbours)
                future_risk = 0
                index = solution[agent].index(item)
                if index < len(solution[agent])-1:
                    next_state = solution[agent][index+1]
                    try:
                        risk_1 = self.env.risk[(item.location.x, item.location.y)][item.time+1]
                    except:
                        risk_1 = 0

                    try:
                        risk_2 = self.env.risk[(next_state.location.x, next_state.location.y)][item.time]
                    except:
                        risk_1 = 0

                    future_risk += (risk_1 * risk_2) 
                safety_value[agent] += future_risk

            safety += safety_value[agent]

        return safety


    def reconstruct_path(self, current):## to be modified

        path = copy.deepcopy(current.path)

        path_list = []
    
        path_list.append(current.path)

        while (current.parent):
            path_list.append(current.parent.path)
            #path_node.update({current.parent.q:current.parent.path})
            for agent in current.parent.path:
                for step in reversed(current.parent.path[agent]):
                    if step not in path[agent]:
                        path[agent].insert(0,step)
            current = copy.deepcopy(current.parent)


        return path, path_list## [::-1]copy from last one to first one

    def search(self):

        node = copy.deepcopy(self.root)
        s_time = time.time()
        open_set = {node}
        closed_set = set()
        safety_value = {}
        
        
        while open_set:
            temp_dict = {open_item:safety_value.setdefault(open_item, float(0)) for open_item in open_set}### to be modified
            current = min(temp_dict, key=temp_dict.get)##return minum value 

            if current.q == self.q_accept: # arrive goal, tasks finished

                return self.reconstruct_path(current)# to be modified

            open_set -= {current} #remove current state from open set

            closed_set |= {current.q} #add current state to closed set

            q_list = self.get_children(current.q) # to be modified


            children = []
            temp_q_list = set()

            for q in q_list:
                if q in closed_set:
                    continue

                for node in open_set:
                    temp_q_list |= {node.q}#add current state to close set

                if q not in temp_q_list:                    
                    sub_node = Node()
                    sub_node.q = q
                    sub_node.state = current.state 
                    sub_node.parent = current
                    sub_node.path =[]
                    sub_node.value = 0
                else:
                    for node in open_set:
                        if node.q == q:
                            sub_node = node

                temp_state = copy.deepcopy(current.state)
                for agent in temp_state:
                    temp_state[agent] = self.map_q_state(current.q,sub_node.q)

            
                temp_path = self.env.compute_solution_risk(current.state, temp_state)
                

                #print("current.value", current.value)
                temp_value = current.value + self.calculate_safety_value(temp_path)*100 + 0*1/len(temp_path)
                #temp_value = current.value + self.calculate_safety_value(temp_path) 
                #print("temp_value",temp_value)
                #input()
                safety_value[sub_node] = temp_value


                if sub_node not in open_set:
                    open_set |= {sub_node}
                elif temp_value > sub_node.value:
                    continue

                #input()
                sub_node.state = temp_state
                sub_node.path = temp_path
                sub_node.value = temp_value
                sub_node.parent = current

                children.append(sub_node)

                for agent in sub_node.state:
                    sub_node.state[agent] = copy.deepcopy(temp_path[agent][-1])

                if sub_node.q == self.q_accept:
                    self.accept_node = copy.deepcopy(sub_node)

            current.add_children(children)

        return False



