import math

class Agent:
    def __init__(self,id,comm_range,posx=0,posy=0):
        self.id = id
        self.x = posx
        self.y = posy
        self.comm_range = comm_range # if robot position is within comm_range, they can comunicate
            
    def __str__(self):
        return f"Robot {self.id} -> ({self.x:.2f}, {self.y:.2f})"

    # Position methods
    def set_pos(self,x,y):
        self.x = x
        self.y = y 

    def get_pos(self):
        return self.x, self.y
    
    # Movement functions
    def motion_planner():
        #######
        return  

    # Goal functions
    def goal_selector():
        #######
        return
    
