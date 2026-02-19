import random
import threading

class Robot:
    def __init__(self,_id,posx=0,posy=0,n=None):

        if n is None:
            n = []  

        self.id = _id
        self.x = posx
        self.y = posy
        self.neighbors = n

        # Lock for thread synchronization
        self.lock = threading.Lock()


    def __str__(self):
        return f"Robot {self.id} -> ({self.x:.2f}, {self.y:.2f})"

    def set_pos(self,x,y):
        with self.lock:  # Ensure thread-safety when updating position
            self.x = x
            self.y = y 

    def get_pos(self):
        with self.lock:  # Ensure thread-safety when reading position
            return self.x, self.y

    def set_neighbors(self, nodes):
        with self.lock:
            self.neighbors = nodes

    def add_node(self, node):
        with self.lock:
            self.neighbors.append(node)

    def remove_node(self, node):
        with self.lock:
            if node in self.neighbors:
                self.neighbors.remove(node)
            else:
                print(f"Element {node} not found in the list.")

    def gossip_update(self):
        if not self.neighbors:
            return
        
        # Gossip rule
        with self.lock:
            neighbor = random.choice(self.neighbors)
            nx, ny = neighbor.get_pos()
            new_x = (nx + self.x) / 2
            new_y = (ny + self.y) / 2

            # Update
            self.set_pos(new_x, new_y)
            neighbor.set_pos(new_x, new_y)

    def gossip_line(self, bx, by):
        if not self.neighbors:
            return
        
        # Gossip rule
        with self.lock:
            neighbor = random.choice(self.neighbors)
            nx, ny = neighbor.get_pos()
            new_x = ((nx - bx * neighbor.id) + (self.x - bx * self.id)) / 2
            new_y = ((ny - by * neighbor.id) + (self.y - by * self.id)) / 2

            # Update
            self.set_pos(new_x + bx * self.id, new_y + by * self.id)
            neighbor.set_pos(new_x + bx * neighbor.id, new_y + by * neighbor.id)


# Simulation with threading
def simulate_robot_behavior(robot):
    # Run gossip update in a loop for each robot
    for _ in range(10):  # Let's simulate 10 iterations
        robot.gossip_update()
        print(robot)                  
                    

def main():
    # Create robots
    robot0 = Robot(0, 0, 0)
    robot1 = Robot(1, 5, 5, [robot0])
    robot2 = Robot(2, 10, 10, [robot0])
    
    # Set neighbors
    robot0.set_neighbors([robot1, robot2])
    
    robots = [robot0, robot1, robot2]
    
    # Create and start threads for each robot
    threads = []
    for robot in robots:
        thread = threading.Thread(target=simulate_robot_behavior, args=(robot,))
        threads.append(thread)
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()


#Test class
if __name__ == "__main__":
    main()    