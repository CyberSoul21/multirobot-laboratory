import numpy as np
import random
import math

class Robot:
    def __init__(self, _id, x=0, y=0):
        self.id = _id  # Robot's unique identifier
        self.x = x      # Position in x-axis
        self.y = y      # Position in y-axis




def points_around(center, n: int, radius: float = 1.0, rotation: float = 0.0):

    cx = center[0,0]
    cy = center[1,0]
    rotation_rad = math.radians(rotation)
    c_around = np.zeros((2,n)) #Create matrix

    for i in range(n):
        c_around[0,i] = cx + radius * math.cos(2 * math.pi * i / n + rotation_rad)
        c_around[1,i] = cy + radius * math.sin(2 * math.pi * i / n + rotation_rad)

    return c_around





Num_robots = 4; # number of bots #square


robots = []
q = np.zeros((2,Num_robots)) #Create matrix
c = np.zeros((2,Num_robots)) #Create matrix
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

#Desired formastion, TODO: create a class called shape
# Square (4 points), rotated 45° so sides are axis-aligned
c = points_around(target, n=Num_robots, radius=1.0, rotation=45) 




A = np.dot(c.transpose(),q)
#A = q @ c.T




#U,S,Vt = np.linalg.svd(A,full_matrices=True)
U,S,Vt = np.linalg.svd(A,full_matrices=True)




V = Vt.T

VUt = np.dot(V,U.transpose())
#VtU = V @ U.T

d = np.sign(np.linalg.det(VUt))



# Build D matrix
D = np.eye(Num_robots)
D[-1, -1] = d

# Compute R
R = V @ D @ U.T


"""
print("Q: ")
print(q)

print("target: ")
print(target)

print("C: ")
print(c)
print("A: ")
print(A)

print("U: ")
print(U)

print("S: ")
print(S)

print("Vt: ")
print(Vt)

print("V: ")
print(V)

print("d: ")
print(d)  

print("D: ")
print(D)  

print("R: ")
print(D)  
"""






