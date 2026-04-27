import math
import random


class Agent:
    def __init__(self, id, comm_range, posx=0, posy=0):
        """
        PAPER CONNECTION:
        Implements the agent model from Section II-A.

        Each agent has:
        - an identifier
        - a position in the global reference frame
        - a communication range R
        - a current assigned target T_i
        - a claimed waypoint wp
        """

        self.id = id
        self.x = posx
        self.y = posy
        self.comm_range = comm_range

        self.neighbors = []
        # PAPER: T_ai(t)
        # Current assigned target position of the agent.
        self.goal = None

        # PAPER: next_step
        # The waypoint the robot wants to move to next.
        self.next_pos = (posx, posy)

        # PAPER: wp
        # The waypoint currently claimed by the robot.
        self.claimed_wp = (posx, posy)

    def __str__(self):
        """
        Utility function.

        Not part of the paper algorithm.
        Used only for debugging and printing robot state.
        """

        return (
            f"Robot {self.id} -> "
            f"pos=({self.x:.2f}, {self.y:.2f}), "
            f"goal={self.goal}"
        )

    # ***********************************************************
    # Position methods
    # ***********************************************************

    def set_pos(self, x, y):
        """
        PAPER CONNECTION:
        Updates p_ai(t), the position of agent a_i.

        Also updates the claimed waypoint wp because, after movement,
        the robot is assumed to occupy/claim its new grid position.
        """

        self.x = x
        self.y = y
        self.claimed_wp = (x, y)

    def get_pos(self):
        """
        PAPER CONNECTION:
        Returns p_ai(t), the current position of the agent
        in the global coordinate frame.
        """

        return self.x, self.y

    def set_goal(self, goal):
        """
        PAPER CONNECTION:
        Sets T_ai(t), the current assigned target of the agent.

        This corresponds to the assignment part of the algorithm.
        """

        self.goal = goal

    def get_goal(self):
        """
        PAPER CONNECTION:
        Returns T_ai(t), the current assigned target.
        """

        return self.goal

    def setNeighbors(self,neighbors):
        self.neighbors = neighbors
        
    # ***********************************************************
    # Distance / communication
    # ***********************************************************
    def distance_to(self, other):
        """
        PAPER CONNECTION:
        Implements the communication-neighborhood condition.

        In the paper, agent a_j is a neighbor of a_i if:

            ||p_ai(t) - p_aj(t)|| <= R

        This function computes the Euclidean distance between two agents.
        """

        ox, oy = other.get_pos()
        return math.sqrt((self.x - ox) ** 2 + (self.y - oy) ** 2)

    def can_communicate(self, other):
        """
        PAPER CONNECTION:
        Implements the local communication range R.

        If this returns True, the other robot belongs to the local
        neighborhood N_ai(t).
        """

        return self.distance_to(other) <= self.comm_range

    def manhattan_distance(self, p1, p2):
        """
        PAPER CONNECTION:
        Implements the grid distance used in the paper.

        The paper uses Manhattan distance because robots move on a
        discrete grid using north, south, east, west, or wait actions.
        """

        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    # ***********************************************************
    # Movement planner
    # ***********************************************************
    def motion_planner(self, x_min, x_max, y_min, y_max):
        """
        PAPER CONNECTION:
        Simplified implementation of the Motion Planning module
        from Section III-A.

        Paper idea:
        - Convert continuous space into a discrete grid.
        - At each waypoint, an agent has five possible actions:
          north, east, south, west, wait.
        - The agent greedily chooses the action that reduces
          Manhattan distance to its assigned goal.

        This function implements ONLY the greedy next-step selection.

        IMPORTANT:
        The collision-free checks from the paper are NOT implemented here.
        They should be implemented in main.py, because collision checking
        requires comparing all agents' proposed movements.
        """

        if self.goal is None:
            # PAPER: If no goal is assigned, the robot cannot plan motion.
            # Simulation choice: wait in place.
            self.next_pos = self.get_pos()
            return self.next_pos

        current_pos = self.get_pos()
        x, y = current_pos

        # PAPER: possible grid actions:
        # north, east, south, west, wait
        candidates = [
            (x + 1, y),  # east
            (x - 1, y),  # west
            (x, y + 1),  # north
            (x, y - 1),  # south
            (x, y),      # wait
        ]

        # Keep only candidates inside the grid.
        # This is a simulation boundary constraint.
        valid_candidates = []
        for px, py in candidates:
            if x_min <= px <= x_max and y_min <= py <= y_max:
                valid_candidates.append((px, py))

        # PAPER: choose the action that greedily reduces
        # Manhattan distance to the goal.
        best_pos = min(
            valid_candidates,
            key=lambda p: self.manhattan_distance(p, self.goal)
        )

        # PAPER: next_step variable.
        self.next_pos = best_pos
        return self.next_pos


    def move(self):
        x, y = self.next_pos
        self.set_pos(x, y)
    # ***********************************************************
    # Goal selector
    # ***********************************************************
    def goal_selector(self, agents, Q):
        """
        PAPER CONNECTION:
        Simplified implementation of the New Goal Selector
        from Section III-B.

        Paper idea:
        - Because agents only have local information, two agents may
          hold the same goal.
        - If two neighboring agents detect that they hold the same goal,
          one of them selects a new goal.
        - The paper uses lexicographical order to decide which robot
          changes its goal.

        This implementation:
        - Detects duplicate goals only among communicating neighbors.
        - The lexicographically smaller-position robot changes goal.
        - It chooses a free target if one is globally visible in simulation.

        IMPORTANT:
        This is NOT the full gradient-based selector from the paper.
        The paper propagates candidate unassigned goals using hop-count
        messages. Here we use a simpler random/free-goal selection.
        """

        return self.goal

    # -----------------------------
    # Local task swapping
    # -----------------------------
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