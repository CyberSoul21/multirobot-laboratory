#-----------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
#-----------------------------------
import random
import threading

class Robot:
    def __init__(self,_id,posx=0,posy=0, bx=0, by=0, n=None):

        if n is None:
            n = []  

        self.id = _id
        self.x = posx
        self.y = posy
        self.neighbors = n
        self.bx = bx
        self.by = by
        self.t_local = random.choice([0.3, 0.6, 0.9, 1.2, 1.5])
        # Manage concurency
        self.lock = threading.RLock() # Allows the same thread to reenter in the lock

    def __str__(self):
        return f"Robot {self.id} -> ({self.x:.2f}, {self.y:.2f}), update time: {self.t_local}"

    def set_pos(self,x,y):
        with self.lock:
            self.x = x
            self.y = y 

    def get_pos(self):
        with self.lock:
            return self.x, self.y
 
    def setNeighbors(self,nodes):
        self.neighbors = nodes

    def addNode(self,node):
        self.neighbors.append(node)

    def removeNode(self,node):
        if node in self.neighbors:
            self.neighbors.remove(node)
        else:
            print(f"Element {node} not found in the list.")

    def gossip_step(self):
        if not self.neighbors:
            return
        
        neighbor = random.choice(self.neighbors)
        first, second = (self, neighbor) if self.id < neighbor.id else (neighbor, self)

        with first.lock:
            with second.lock:
                # Gossip rule
                nx,ny = neighbor.get_pos()
                new_x = ((nx - self.bx*neighbor.id) + (self.x - self.bx*self.id)) / 2
                new_y = ((ny - self.by*neighbor.id) + (self.y - self.by*self.id)) / 2
                
                # Update
                self.set_pos(new_x + self.bx*self.id, new_y + self.by*self.id)
                neighbor.set_pos(new_x + self.bx*neighbor.id, new_y + self.by*neighbor.id)      
                    
