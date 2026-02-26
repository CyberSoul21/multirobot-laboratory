import numpy as np
import random


class Robot:
    def __init__(self, _id, x=0, y=0):
        self.id = _id  # Robot's unique identifier
        self.x = x      # Position in x-axis
        self.y = y      # Position in y-axis



Num_robots = 5; # number of bots


robots = []
q = np.zeros((2,Num_robots)) #Create matrice
c = np.zeros((2,Num_robots)) #Create matrice
target = np.array([[random.randint(0,10)],[random.randint(0,10)]])
d = 1

for i in range(Num_robots):
    # Random initial positions
    x = random.randint(0, 10)
    y = random.randint(0, 10)
    q[0,i] = x
    q[1,i] = y
    robots.append(Robot(i, x, y))

    #c[0,i] =  random.randint(0, 10) 

c = d*(target - 1)

print(c)

         


