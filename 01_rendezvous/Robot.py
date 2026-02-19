class Robot:
    def __init__(self,_id,posx=0,posy=0,n=None):

        if n is None:
            n = []  

        self.id = _id
        self.x = posx
        self.y = posy
        self.neighbors = n


    def __str__(self):
        pass

    def setX(self,x):
        self.x = x

    def setY(self,y):
        self.y = y   

    def getX(self):
        return self.x

    def getY(self):
        return self.y

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
        # Aux variables to accumulate new x and y positions
        sum_x = 0
        sum_y = 0
        
        # Iterate through all neighbors and accumulate their positions
        for neighbor in self.neighbors:
            sum_x += neighbor.x
            sum_y += neighbor.y
        
        # Calculate the average position from all neighbors
        avg_x = sum_x / len(self.neighbors)
        avg_y = sum_y / len(self.neighbors)
        
        # Update the robot's position based on the average
        self.x = avg_x
        self.y = avg_y            
                    

#TODO: Test class,  add comments, and methods to plot..or create another class to plot?
# Test class if this script is executed directly
def main():
    # Create an instance of the Calculator class
    robot0 = robot()


#Test class
if __name__ == "__main__":
    main()    