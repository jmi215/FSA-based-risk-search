# encoding: utf-8
from map import Map
from constraint import Constraints, EdgeConstraint, VertexConstraint, Conflict
from environment import Environment
import copy
import yaml
import time
import os
import sys
import numpy as np
import math
from math import fabs
import matplotlib.pyplot as plt
import seaborn
from state import State, Location
import argparse
from graph import Graph


def main(map_type):
	# initialize the environment
	info = Map(map_type)
	env = Environment(info.dimension, info.agents, info.humans, info.obstacles, info.Num_agents, info.Num_humans)
	#env.make_agent_dict()
	goal_N = 0

	# initial state
	initial_state_agent = {}
	for agent in env.agent_dict.keys():
		initial_state_agent.update({agent:env.agent_dict[agent]['current']})
		goal_N += len(env.agent_dict[agent]['goal'])

	state_agent = copy.deepcopy(initial_state_agent)


	initial_state_human = {}
	for human in env.human_dict.keys():
		initial_state_human.update({human:env.human_dict[human]['current']})
	state_human = copy.deepcopy(initial_state_human)

	
	# plan of agents and humans
	plan = {}
	for agent in env.agent_dict.keys():
		plan.update({agent:[]})
	for human in env.human_dict.keys():
		plan.update({human:[]})


	env.TH_conflict={}
	conflict_within_TH = []
	graph = Graph(state_agent, state_human, env)

	# risk creating
	print("create human risk constraints")
	real_path_human = {}
	env.human_simulation()
	
	### search 
	print("Starting search safe path")
	initial_q = 3
	start_time = time.time()
	plan, path_list = graph.search()
	end_time = time.time()
	computational_time = end_time - start_time

		
	# get the length of the plan
	LEN = 0
	for agent in plan:
		if len(plan[agent]) > LEN:
			LEN = len(plan[agent])


	# observed path of the human			
	for human in env.human_dict.keys():
		env.human_dict[human]["current"] = copy.deepcopy(state_human[human])
		
	tem_path = env.sumulator.simulation(LEN-1)

	for human in tem_path:
		real_path_human.update({human:tem_path[human]})
	ob_path_human = copy.deepcopy(real_path_human)
	

	# add observed human path
	for human in ob_path_human:
		plan.update({human:ob_path_human[human]})

	# get index_list in time step order
	index_list = []
	for path in path_list:
		for agent in path:
			index_1, index_2 = plan[agent].index(path[agent][0]), plan[agent].index(path[agent][-1])
			index_list.append((index_1, index_2))

	index_list_p = sorted(index_list, key=lambda x:x[0], reverse=False)

	tem_plan = {}

	for t in range(env.Max_step):
		env.step_conflict.update({t:0})

	reward = 0
	goal = 0
	conflict_num = 0

	for item in index_list_p:
		for agent in plan:
			tem_plan.update({agent:plan[agent][item[0]:item[1]+1]})

		env.plan = copy.deepcopy(tem_plan)

		r_ep, goal_ep, conflict_num_ep, step_conflict = env.count_conflict_reward()
		reward += r_ep
		if conflict_num_ep == 0 and conflict_num==0:
			goal += goal_ep
		conflict_num += conflict_num_ep


	path = open("log_path/path.txt", "w")
	for agent in plan:
		path.write(str(agent)+"\t")
		for step in range(len(plan[agent])):
			path.write(str(plan[agent][step])+"\t")
		path.write(str("\n"))
	path.close()

	return reward, goal, conflict_num, step_conflict, goal_N, LEN, computational_time


if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser = argparse.ArgumentParser(description='input:map_type')
	parser.add_argument('map_type', metavar='map_type', type=int, help='map_type')


	args = parser.parse_args()
	map_type = args.map_type


	r, goal_achieved, conflict_num, step_conflict, goal_N, length, tim = main(map_type)

	print("reward", round(r,3), "conflict_num", conflict_num, "achieved_goals", goal_achieved, "Computation time", tim)




