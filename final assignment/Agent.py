import math
import random

class Agent:
    def __init__(self, id, comm_range, posx=0, posy=0):
        self.id = id
        self.pos = (posx, posy)
        self.actual_wp = (posx, posy)
        self.next_wp = (posx, posy)
        self.comm_range = comm_range
        self.goal = None
        self.neighbors = []
        self.validMovement_ = False
        #Gradient
        self.candidate_goal = None
        self.hop = float("inf")

    def __str__(self):
        x, y = self.pos
        return (
            f"Robot {self.id} -> "
            f"pos=({x:.2f}, {y:.2f}), "
            f"goal={self.goal}"
        )

    # Position methods
    def get_id(self):
        return self.id
    
    def set_pos(self, x, y):
        self.pos = (x, y)
        
    def get_pos(self):
        return self.pos
    
    def set_actual_wp(self, x, y):
        self.actual_wp = (x, y)
    
    def get_actual_wp(self):
        return self.actual_wp
    
    def set_next_wp(self, x, y):
        self.next_wp = (x, y)

    def get_next_wp(self):
        return self.next_wp

    def set_goal(self, goal):
        self.goal = goal

    def get_goal(self):
        return self.goal

    def setNeighbors(self,neighbors):
        self.neighbors = neighbors


    # Distance / communication
    def distance_to(self, other):
        # Computes Euclidean distance
        ox, oy = other.get_pos()
        pos_x, pos_y = self.pos
        return math.sqrt((pos_x - ox) ** 2 + (pos_y - oy) ** 2)

    def can_communicate(self, other):
        return self.distance_to(other) <= self.comm_range

    def manhattan_distance(self, p1, p2):
        # Computes Manhattan distance
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def lex_greater(self, pos_a, pos_b):
        # Returns true if pos_a is lex greater than pos_b
        if pos_a[0] != pos_b[0]:
            return pos_a[0] > pos_b[0]
        return pos_a[1] > pos_b[1]

    # Movement planner
    def is_at_waypoint(self, pos, wp, tol=1e-9):
        return abs(pos[0] - wp[0]) < tol and abs(pos[1] - wp[1]) < tol
    
    def motion_planner(self, x_min, x_max, y_min, y_max):
        
        # This is computed when an agent is at a waypoint
        if self.is_at_waypoint(self.pos, self.next_wp):
            pos_x, pos_y = self.pos
            self.set_actual_wp(round(pos_x), round(pos_y))

            # Possible actions inside the grid: north, east, south, west, wait
            x, y = self.get_actual_wp()
            candidates = [
                (x + 1, y),  # east
                (x - 1, y),  # west
                (x, y + 1),  # north
                (x, y - 1),  # south
                (x, y),      # wait
            ]

            # Grid boundary constraint.
            valid_candidates = []
            for px, py in candidates:
                if x_min <= px <= x_max and y_min <= py <= y_max:
                    valid_candidates.append((px, py))

            # Greedy action
            best_pos = min(
                valid_candidates,
                key=lambda p: self.manhattan_distance(p, self.goal)
            )

            self.next_wp = best_pos

    #TODO: Issue, the robot has the target but on the way toward the target
    #it might pass over one target, as the agent has the entire map it should
    # be able to compute if it is more optime change the goal by the closest to itself 
    
    #TODO: plot the trayectory computed!!
    def move(self):
        # This is replicating the communication before movement
        valid_movement = True
        for neighbor in self.neighbors:
            neighbor_actual_wp = neighbor.get_actual_wp()
            neighbor_next_wp = neighbor.get_next_wp()

            # Constraint III-A.1a Neighbor is in next wp
            if self.next_wp == neighbor_actual_wp:
                valid_movement = False
                print("SWAP 1 !")
                break

            # Constraint III-A.1b Only moves lexicographically greater to next_wp
            if self.next_wp == neighbor_next_wp:
                if not self.lex_greater(self.actual_wp, neighbor_actual_wp):
                    valid_movement = False
                    print("SWAP 2 !")
                    break

            # Constraint III-A.2 Do not travel same edge
            if self.next_wp == neighbor_actual_wp and self.actual_wp == neighbor_next_wp:
                valid_movement = False
                print("SWAP! 3 ")
                break

        # Final action
        if valid_movement:
            next_wp_x, next_wp_y = self.next_wp
            actual_wp_x, actual_wp_y = self.actual_wp
            pos_x, pos_y = self.pos
            x = pos_x + 0.1*(next_wp_x-actual_wp_x)
            y = pos_y + 0.1*(next_wp_y-actual_wp_y)
        else: 
            x, y = self.get_pos() # wait

        self.set_pos(x, y)
        self.validMovement_ = valid_movement


    # Goal selector
    # 
    
    def goal_selector(self, agents, Q):
        if self.goal is None:
            self.goal = random.choice(Q)
            return self.goal

        # Always define this first
        neighbor_goals = [
            other.goal
            for other in agents
            if other.id != self.id and self.can_communicate(other)
        ]

        for other in agents:
            if other.id == self.id:
                continue

            if not self.can_communicate(other):
                continue

            if self.goal == other.goal:
                if self.get_pos() < other.get_pos():

                    if self.candidate_goal is not None and self.candidate_goal not in neighbor_goals:
                    #if self.candidate_goal is not None:
                        self.goal = self.candidate_goal
                    else:
                        free_goals = [q for q in Q if q not in neighbor_goals]

                        if free_goals:
                            self.goal = random.choice(free_goals)
                        else:
                            self.goal = random.choice(Q)

        return self.goal    
    
    # def update_gradient_candidate(self, Q):
    #     """
    #     PAPER CONNECTION:
    #     Gradient-based selector from Section III-B.

    #     This approximates the hop-count propagation mechanism:
    #     - If the agent is near an apparently unassigned goal, it becomes an anchor.
    #     - Anchor publishes that goal with hop = 0.
    #     - Otherwise, the agent copies the best candidate_goal from neighbors
    #     with the smallest hop count.
    #     """

    #     # Goals currently visible among local neighbors
    #     neighbor_goals = [neighbor.get_goal() for neighbor in self.neighbors]

    #     # Check if there is a locally unassigned goal one grid step away
    #     local_unassigned_goals = []

    #     for q in Q:
    #         if q not in neighbor_goals:
    #             if self.manhattan_distance(self.get_actual_wp(), q) == 1:
    #                 local_unassigned_goals.append(q)

    #     # Anchor case
    #     if local_unassigned_goals:
    #         self.candidate_goal = random.choice(local_unassigned_goals)
    #         self.hop = 0
    #         return

    #     # Common-agent case: receive best candidate from neighbors
    #     best_neighbor = None
    #     best_hop = float("inf")

    #     for neighbor in self.neighbors:
    #         if neighbor.candidate_goal is not None and neighbor.hop < best_hop:
    #             best_neighbor = neighbor
    #             best_hop = neighbor.hop

    #     if best_neighbor is not None:
    #         self.candidate_goal = best_neighbor.candidate_goal
    #         self.hop = best_neighbor.hop + 1
    #     else:
    #         self.candidate_goal = None
    #         self.hop = float("inf")


    def update_gradient_candidate(self, Q):
        """
        Improved gradient-based selector.

        Keeps candidate_goal persistent unless a better candidate is found.
        """

        neighbor_goals = [neighbor.get_goal() for neighbor in self.neighbors]

        # 1. Anchor case: goal one grid step away and not used by neighbors
        local_unassigned_goals = []

        for q in Q:
            if q not in neighbor_goals:
                if self.manhattan_distance(self.get_actual_wp(), q) == 1:
                    local_unassigned_goals.append(q)

        if local_unassigned_goals:
            # Choose closest local candidate deterministically
            self.candidate_goal = min(
                local_unassigned_goals,
                key=lambda q: self.manhattan_distance(self.get_actual_wp(), q)
            )
            self.hop = 0
            return

        # 2. Propagation case: receive best candidate from neighbors
        best_candidate = self.candidate_goal
        best_hop = self.hop

        for neighbor in self.neighbors:
            if neighbor.candidate_goal is None:
                continue

            candidate = neighbor.candidate_goal
            hop = neighbor.hop + 1

            # Accept only if better than current memory
            if hop < best_hop:
                best_candidate = candidate
                best_hop = hop

        self.candidate_goal = best_candidate
        self.hop = best_hop            


    # Local task swapping
    def try_goal_swap(self, other, beta=0.1):
        """
        PAPER CONNECTION:
        Implements the Local Task Swapping idea from Section III-A
        and Algorithm 4.

        Paper idea:
        - Neighboring agents compare the total pairwise Manhattan distance
          before and after swapping goals.
        - If swapping decreases total distance, they swap goals.
        - If swapping keeps the same distance, they swap with probability beta.
          This helps resolve deadlocks.

        This function implements:
        - improving swap
        - neutral probabilistic swap

        IMPORTANT:
        The paper uses a 2-way handshake to guarantee that only one
        consistent pairwise swap happens at a time.
        This function does NOT implement the full handshake protocol.
        That should be approximated in main.py by preventing duplicate
        simultaneous swaps.
        """

        if self.goal is None or other.goal is None:
            return False

        # PAPER: current pairwise cost:
        # d(self, self.goal) + d(other, other.goal)
        current_cost = (
            self.manhattan_distance(self.get_pos(), self.goal)
            + self.manhattan_distance(other.get_pos(), other.goal)
        )

        # PAPER: swapped pairwise cost:
        # d(self, other.goal) + d(other, self.goal)
        swapped_cost = (
            self.manhattan_distance(self.get_pos(), other.goal)
            + self.manhattan_distance(other.get_pos(), self.goal)
        )

        # PAPER: swap if total pairwise distance decreases.
        if swapped_cost < current_cost:
            self.goal, other.goal = other.goal, self.goal
            return True

        # PAPER: if cost is equal, swap with probability beta.
        # This is used to help break deadlocks.
        if swapped_cost == current_cost and random.random() < beta:
            self.goal, other.goal = other.goal, self.goal
            return True

        return False    
    

if __name__ == "__main__":
    print("test")