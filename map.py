
class Map(object):
    def __init__(self, map_type):


        if map_type == 7:
            #grid size
            self.N_x = 7
            self.N_y = 7
            self.dimension = [self.N_x, self.N_y]    

            #number of agents,human
            self.Num_agents = 1
            self.Num_humans = 1

            #static obstacles
            self.obstacles = [(1,1),(1,2),(1,3),(1,4),\
                            (3,2),(3,3),(3,4),(3,5),\
                            (5,1),(5,2),(5,3),(5,4)]

            self.agents = [{'start': [1, 0], 'goal': [(5,5),(1,5),(3,1)], 'name': 'agent0'}]
            self.humans = [{'start': [3, 6], 'goal': [None,None],'name': 'human0'}]


        if map_type == 10:
            #grid size
            self.N_x = 10
            self.N_y = 10
            self.dimension = [self.N_x, self.N_y]    

            #number of agents,human
            self.Num_agents = 1
            self.Num_humans = 1

            #static obstacles
            self.obstacles = []
            for x in [1,3,6,8]:
                for y in [1,2,3,6,7,8]:
                    self.obstacles.append((x,y))

            self.agents = [{'start': [1, 0], 'goal': [(1,8),(6,1),(8,6)], 'name': 'agent0'}]

            for item in [(1,8),(6,1),(8,6)]:
                if item in self.obstacles:
                    self.obstacles.remove(item)
            #self.agents = [{'start': [1, 0], 'goal': [(5,5)], 'name': 'A0'}]
            self.humans = [{'start': [4, 5], 'goal': [None,None],'name': 'human0'}]


        if map_type == 20:
            #grid size
            self.N_x = 20
            self.N_y = 20
            self.dimension = [self.N_x, self.N_y]    

            #number of agents,human
            self.Num_agents = 1
            self.Num_humans = 1

            #static obstacles
            self.obstacles = []
            for x in [1,3,5,7,9,12,14,16,18]:
                for y in [1,2,3,4,5,6,7,8,11,12,13,14,15,16,17,18]:
                    self.obstacles.append((x,y))

            self.agents = [{'start': [1, 0], 'goal': [(3,18),(12,1),(18,18)], 'name': 'agent0'}]

            for item in self.agents:
                for goal in item["goal"]:
                    if goal in self.obstacles:
                        self.obstacles.remove(goal)
            self.humans = [{'start': [10, 10], 'goal': [None,None],'name': 'human0'}]


