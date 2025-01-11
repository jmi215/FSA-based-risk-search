import random
from state import State, Location
import copy
from itertools import combinations

class Simulator():
	def __init__(self,env):
		self.agent_dict =  env.agent_dict
		self.env =  env
		self.get_neighbors = env.get_neighbors
		self.TH = env.TH
		self.constraints = env.constraints
		self.obstacles = env.obstacles


	def simulation(self,step):
		
		plan = {}
		tem = {}
		state_human = {}
		
		for human in self.env.human_dict.keys():
			plan.update({human:[]})
			plan[human].append(self.env.human_dict[human]["current"])
			state_human.update({human:self.env.human_dict[human]["current"]})

		for i in range(step):
			neighbour_set = {}
			next_state = {}
			for human in self.env.human_dict.keys():
				neighbors = self.env.get_legal_neighbors_human(state_human, human)
				neighbour_set.update({human:neighbors})

			human_list = []
			for human in self.env.human_dict.keys():
				human_list.append(human)

			while len(human_list)>0:
				human = random.choice(human_list)
				tem_neighbours = copy.deepcopy(neighbour_set[human])
				state_human[human] = copy.deepcopy(random.choice(tem_neighbours))
				for item in next_state:
					while(state_human[human].__eq__(next_state[item])):
						tem_neighbours.remove(state_human[human])
						state_human[human] = copy.deepcopy(random.choice(tem_neighbours))

				next_state.update({human:state_human[human]})
				plan[human].append(state_human[human])

				human_list.remove(human)
        
		return plan
	

	
	



