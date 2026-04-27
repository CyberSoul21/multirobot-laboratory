import math

class Agent:
    def __init__(self,id,comm_range,posx=0,posy=0):
        self.id = id
        self.x = posx
        self.y = posy
        self.comm_range = comm_range # if robot position is within comm_range, they can comunicate

        self.neighbors = []
            
    def __str__(self):
        return f"Robot {self.id} -> ({self.x:.2f}, {self.y:.2f})"

    # Position methods
    def set_pos(self,x,y):
        self.x = x
        self.y = y 

    def get_pos(self):
        return self.x, self.y
    
    def is_in_range(self, pos):
        other_x, other_y = pos
        if (abs(math.sqrt(self.x**2 + self.y**2) - math.sqrt(other_x**2 + other_y**2)) <= self.comm_range):
            return True
        return False
    
    def setNeighbors(self,neighbors):
        self.neighbors = neighbors

    # Movement functions
    def motion_planner():
        """ Constraint III-A.1: At any time t, no agent moves to the waypoint that is currently occupied by the other, and no pair
                                of agents move toward the same waypoint at the same time.
            Constraint III-A.2: At any time t, no pair of agents travel on the same edge in opposite directions.
            For each agent located at any waypoint, there are five possible actions -> north, east, south, west, and wait."""
        #######
        return

    def move(): 
        #######
        return 

    # Goal functions
    def goal_selector():
        """ In the paper there is a handshake to ensure concurrency, here would not be possible, but we can still do it
            1) Random Selector:
            2) Gradient-Based Selector:
        """
        #######
        return
    

def simulation_step():
    #Listen-think-walk manner
    for agent in A:
        neighbors = []
        for neighbor in A:
            if(neighbor is not agent and agent.is_in_range(neighbor.get_pos())):
                neighbors.append(neighbor)
        agent.setNeighbors(neighbors = neighbors)


if  __name__ == "__main__":
    num_agents = 5
    A = [] # set of agents
    for a in range(num_agents):
        A.append(Agent(id = a, comm_range = 5, posx=a, posy=a)) # I guess we will define an agent class

    simulation_step()