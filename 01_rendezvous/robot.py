class robot:
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
                   

#TODO: Test class,  add comments, and methods to plot..or create another class to plot?
# Test class if this script is executed directly
def main():
    # Create an instance of the Calculator class
    robot0 = robot()


#Test class
if __name__ == "__main__":
    main()    