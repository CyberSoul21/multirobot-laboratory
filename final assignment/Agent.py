import math
import random

class Agent:
    def __init__(self, id, comm_range, posx=0, posy=0):
        # Position
        self.id = id
        self.pos      = (float(posx), float(posy))
        self.actual_wp = (int(posx),   int(posy))   
        self.next_wp   = (int(posx),   int(posy))           
        self.goal = None
        
        # Comunication
        self.comm_range = comm_range
        self.neighbors = []
        self.validMovement_ = False
        self.valid_movement = True 

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

    #---- Position methods ----#
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


    #---- Distance / communication ----#
    # Computes Euclidean distance
    def distance_to(self, other):
        ox, oy = other.get_pos()
        pos_x, pos_y = self.pos
        return math.sqrt((pos_x - ox) ** 2 + (pos_y - oy) ** 2)

    def can_communicate(self, other):
        return self.distance_to(other) <= self.comm_range

    # Computes Manhattan distance
    def manhattan_distance(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    # Returns true if pos_a is lex greater than pos_b
    def lex_greater(self, pos_a, pos_b):
        if pos_a[0] != pos_b[0]:
            return pos_a[0] > pos_b[0]
        return pos_a[1] > pos_b[1]

    #---- Movement planner ----#
    def is_at_waypoint(self, pos, wp, tol=1e-3):
        return abs(pos[0] - wp[0]) < tol and abs(pos[1] - wp[1]) < tol
    
    def motion_planner(self, x_min, x_max, y_min, y_max):

        # Replan if: arrived at next_wp  OR  was blocked last tick
        # This prevents the robot freezing when decide_move() blocks it
        at_next = self.is_at_waypoint(self.pos, self.next_wp)
        was_blocked = not self.valid_movement  # False if decide_move blocked us

        if at_next or was_blocked:

            # Only update actual_wp when truly arrived at a waypoint.
            # When blocked mid-step, keep actual_wp as last confirmed integer position
            # so commit_move computes a clean delta and robot doesn't drift off-grid.
            if at_next:
                pos_x, pos_y = self.pos
                self.set_actual_wp(round(pos_x), round(pos_y))

            # Possible actions: north, east, south, west, wait
            x, y = self.get_actual_wp()

            candidates = [
                (x + 1, y),  # east
                (x - 1, y),  # west
                (x, y + 1),  # north
                (x, y - 1),  # south
                (x, y),      # wait
            ]

            # Grid boundary constraint
            valid_candidates = [
                (px, py) for px, py in candidates
                if x_min <= px <= x_max and y_min <= py <= y_max
            ]

            # Greedy: pick step that minimises Manhattan distance to goal
            self.next_wp = min(
                valid_candidates,
                key=lambda p: self.manhattan_distance(p, self.goal)
            )        

    def decide_move(self):
        # THINK phase: check neighbours using the current snapshot.
        # Nobody has moved yet this tick, so actual_wp values are consistent.
        self.valid_movement = True
        for neighbor in self.neighbors:
            neighbor_actual_wp = neighbor.get_actual_wp()
            neighbor_next_wp   = neighbor.get_next_wp()

            # Constraint A.1a: neighbour is sitting on our intended waypoint
            if self.next_wp == neighbor_actual_wp:
                self.valid_movement = False
                break

            # Constraint A.1b: neighbour with higher lex priority wants same waypoint
            if self.next_wp == neighbor_next_wp:
                if not self.lex_greater(self.actual_wp, neighbor_actual_wp):
                    self.valid_movement = False
                    break

            # Constraint A.2: head-on edge collision
            if self.next_wp == neighbor_actual_wp and self.actual_wp == neighbor_next_wp:
                self.valid_movement = False
                break

    def commit_move(self, x_min=0, x_max=10, y_min=0, y_max=10):
        # WALK phase: execute the decision made in decide_move().
        if self.valid_movement:
            next_wp_x, next_wp_y     = self.next_wp
            actual_wp_x, actual_wp_y = self.actual_wp
            pos_x, pos_y             = self.pos

            x = pos_x + 0.1 * (next_wp_x - actual_wp_x)
            y = pos_y + 0.1 * (next_wp_y - actual_wp_y)

            # Hard clamp: robot can never leave the grid
            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))

            # SNAP: if close enough to next_wp, lock exactly onto it
            # This prevents robots stopping at 2.9999 or 3.0001 forever
            if abs(x - next_wp_x) < 0.15 and abs(y - next_wp_y) < 0.15:
                x = float(next_wp_x)
                y = float(next_wp_y)

            # If clamped against wall, reset waypoints for clean replan
            raw_x = pos_x + 0.1 * (next_wp_x - actual_wp_x)
            raw_y = pos_y + 0.1 * (next_wp_y - actual_wp_y)
            if raw_x != x or raw_y != y:
                self.set_actual_wp(round(x), round(y))
                self.next_wp = (round(x), round(y))
                self.valid_movement = False

            self.set_pos(x, y)
 
    #---- Goal assignment ----#       
    def goal_selector(self, agents, Q, flag=False):

        # If the agent has no goal yet, assign one randomly from Q
        if self.goal is None:
            self.goal = random.choice(Q)
            return self.goal

        # Build the list of goals currently held by neighbours in comm range
        neighbor_goals = [
            other.goal for other in agents
            if other.id != self.id and self.can_communicate(other)
        ]

        # --- CONFLICT RESOLUTION (paper §III-B) ---
        # Check if any neighbour holds the same goal as this agent
        for other in agents:
            if other.id == self.id:
                continue
            # Only interact with agents within communication range
            if not self.can_communicate(other):
                continue

            # Conflict detected: both this agent and 'other' hold the same goal
            if self.goal == other.goal:

                # Tie-breaking: the agent with the lexicographically SMALLER
                # position must change its goal (the other one keeps it)
                # Paper §II-B: p1 ≻ p2 iff p1.x > p2.x, or p1.x==p2.x and p1.y > p2.y
                if self.get_pos() < other.get_pos():

                    # Gradient-based choice (paper §III-B.2):
                    # Use the candidate_goal propagated by the hop-count wave,
                    # as long as it is not already taken by a visible neighbour                    

                    old_goal = self.goal  # remember current goal

                    if self.candidate_goal is not None and self.candidate_goal not in neighbor_goals:
                        self.goal = self.candidate_goal
                    else:
                        # Random fallback (paper §III-B.1):
                        # Pick any goal not currently held by a visible neighbour                        
                        free_goals = [q for q in Q if q not in neighbor_goals]
                        self.goal = random.choice(free_goals) if free_goals else random.choice(Q)

                    # Goal changed mid-movement → force immediate replan
                    # so motion_planner doesn't keep heading to the old target
                    # if self.goal != old_goal:
                    #     self.next_wp = self.actual_wp
                    #     self.valid_movement = False
                    if self.goal != old_goal:
                        self.pos = (float(self.actual_wp[0]), float(self.actual_wp[1]))
                        self.next_wp = self.actual_wp
                        self.valid_movement = False                        

        # --- IDLE AGENT CHECK ---
        # If this agent has already reached its goal, check whether any
        # goal in Q is completely unclaimed by any agent in the swarm.
        # This handles the case where an agent finishes early while
        # some goal squares are left empty (J2 > 0).
        # Note: this uses global agent list, which is acceptable since
        # all agents share Q a priori (paper §II-C)        
        tol = 1e-3
        px, py = self.get_pos()
        gx, gy = self.goal
        if abs(px - gx) < tol and abs(py - gy) < tol:
            all_claimed = [other.goal for other in agents if other.id != self.id]
            unclaimed = [q for q in Q if q not in all_claimed]

            if unclaimed:
                old_goal = self.goal
                self.goal = min(unclaimed,
                                key=lambda q: self.manhattan_distance(self.get_pos(), q))

                # New goal assigned → force replan toward it
                if self.goal != old_goal:
                # Move toward the closest unclaimed goal (greedy assignment)
                # Minimises the extra distance this agent needs to travel                    
                    # self.next_wp = self.actual_wp
                    # self.valid_movement = False
                    self.pos = (float(self.actual_wp[0]), float(self.actual_wp[1]))
                    self.next_wp = self.actual_wp
                    self.valid_movement = False                    

        return self.goal         

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

    def update_gradient_candidate(self, Q):
        """
        PAPER CONNECTION:
        Gradient-based selector from Section III-B.

        This approximates the hop-count propagation mechanism:
        - If the agent is near an apparently unassigned goal, it becomes an anchor.
        - Anchor publishes that goal with hop = 0.
        - Otherwise, the agent copies the best candidate_goal from neighbors
        with the smallest hop count.
        - Keeps candidate_goal persistent unless a better candidate is found.
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
    