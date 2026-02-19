import random

class Robot:
    def __init__(self,_id,posx=0,posy=0,n=None):

        if n is None:
            n = []  

        self.id = _id
        self.x = posx
        self.y = posy
        self.neighbors = n


    def __str__(self):
        return f"Robot {self.id} -> ({self.x:.2f}, {self.y:.2f})"

    def set_pos(self,x,y):
        self.x = x
        self.y = y 

    def get_pos(self):
        return self.x, self.y

    # Complete list 
    def setNeighbors(self,nodes):
        self.neighbors = nodes

    # add one robot (node)
    def addNode(self,node):
        self.neighbors.append(node)

    # remove one robot (node)
    def removeNode(self,node):
        if node in self.neighbors:
            self.neighbors.remove(node)
        else:
            print(f"Element {node} not found in the list.")

    def gossip_update(self):
        if not self.neighbors:
            return
        
        # Calculate the average position from all neighbors
        neighbour = random.choice(self.neighbors)
        nx,ny = neighbour.get_pos()
        new_x = (nx + self.x) / 2
        new_y = (ny + self.y) / 2
        
        # Update the robot's position based on the average
        self.set_pos(new_x, new_y)
        neighbour.set_pos(new_x, new_y)      

    def gossip_line(self, bx, by):
        if not self.neighbors:
            return
        
        # Calculate the average position from all neighbors
        neighbour = random.choice(self.neighbors)
        nx,ny = neighbour.get_pos()
        new_x = ((nx - bx*neighbour.id) + (self.x - bx*self.id)) / 2
        new_y = ((ny - by*neighbour.id) + (self.y - by*self.id)) / 2
        
        # Update the robot's position based on the average
        self.set_pos(new_x + bx*self.id, new_y + by*self.id)
        neighbour.set_pos(new_x + bx*neighbour.id, new_y + by*neighbour.id)         
                    

#TODO: Test class,  add comments, and methods to plot..or create another class to plot?
# Test class if this script is executed directly
def main():
    # Create an instance of the Calculator class
    robot0 = robot()


#Test class
if __name__ == "__main__":
    main()    