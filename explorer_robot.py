import numpy as np
import random
from dijkstar import Graph, find_path
from abc import abstractmethod

from abstract_agent import AbstractAgent
from physical_agent import PhysAgent
from rescuer import Rescuer

from rescuer import Rescuer
from utils import Direction, DIRECTIONS, LINE_DIRECTIONS, DIAG_DIRECTIONS, Victim

class Explorer_robot (AbstractAgent):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

        # Iniciate attributes
        # self._rescuer: Rescuer = resc

        self.pos = (0,0)
        self.victims: dict[int, Victim] = {}                            # Associate the id to the victim
        self.graph = Graph()                                            # Graph will be used to find the path of the current position to the base position
        self.map: dict[tuple[int, int], int] = {}                       # Associate the position in the map to the: CLEAR = 0   WALL = 1    END = 2
        self.untried: dict[tuple[int, int], list[Direction]] = {}       # Associate the current position to the possible untried directions
        self.backtrack: dict[tuple[int, int], list[Direction]] = {}     # Used in the DFS to return to the previous state


        # First iteration
        self.add_map(dir= Direction.NONE, tile_type= PhysAgent.CLEAR)  # Setting the default values for the first tile
        self.read_outskirts()                                           # Look for the close tiles

    @abstractmethod
    def _bias_xdir(self, delta: float) -> bool:
        """
        Returns whether the agent should bias the x direction towards the given delta.
        """
    pass

    @abstractmethod
    def _bias_ydir(self, delta: float) -> bool:
        """
        Returns whether the agent should bias the y direction towards the given delta.
        """
    pass


    def add_map(self, dir: Direction, tile_type: int) -> None:
        """
        Only for new tiles
        Adds the current tile with the type: CLEAR = 0   WALL = 1    END = 2
        and the position shift: dir
        """
        # Current position
        x, y = self.pos
        # Position shift
        dx, dy = dir.value      # value from the Enum
        # New position
        x += dx
        y += dy

        # Verifies if the tile is already in the robots's map
        if (x, y) in self.map:
            return

        # Add the new tile to the robot's map and associate with the tile_type
        self.map[(x, y)] = tile_type

        # Add this new tile to the untried dict
        self.untried[(x, y)] = DIRECTIONS.copy()

        # Add tile to backtrack for the DFS
        self.backtrack[(x, y)] = []

        # Add the tile to the robot's graph
        for dir in DIRECTIONS:

        # Calculating outskirts positions
            x2, y2 = x, y
            dx, dy = dir.value
            x2 += dx
            y2 += dy
            new_position = (x2, y2)

            # Skip if tile is not in map or if are not the CLEAR type
            if (
                new_position not in self.map
                or self.map[new_position] != PhysAgent.CLEAR    # Check if the tile is a WALL or END
                or tile_type != PhysAgent.CLEAR
            ):
                continue

            # Add the link between the tiles in the Graph
            self.graph.add_edge(
                (x, y),
                new_position,
                self.COST_LINE if dir in LINE_DIRECTIONS else self.COST_DIAG,   # A -> B
            )
            self.graph.add_edge(
                new_position,
                (x, y),
                self.COST_LINE if dir in LINE_DIRECTIONS else self.COST_DIAG,   # B -> A
            )


    def read_outskirts(self) -> None:
        """
        Checks the outskirts tiles and add them to the robot's map
        Updates the Graph
        """
        # Read the close tiles
        outskirts = self.body.check_obstacles()     # Implemented in the 'Physical agent'

        # Add tiles to the map
        for i in range(8):
            self.add_map(dir=DIRECTIONS[i], tile_type=outskirts[i])


    def remove_invalid_actions(self) -> None:
        """
        Removes from the possible actions the tiles that are not not reachable, by any constraint (Battery, Wall, End).
        """
        for dir in self.untried[self.pos].copy():       # Copy because dont want to change the original value yet
            # Position
            x, y = self.pos
            dx, dy = dir.value
            x += dx
            y += dy

            # WALL or END: Remove from the possible actions
            tile_type = self.map[(x, y)]
            if tile_type == PhysAgent.WALL or tile_type == PhysAgent.END:
                self.untried[self.pos].remove(dir)      # Removes from the original value
                continue

            # Calculates the battery if the robot goes to the tile and interacts with the victim
            walk_cost = self.COST_LINE if dir in LINE_DIRECTIONS else self.COST_DIAG
            battery = self.body.rtime - walk_cost

            # Calculates the minimun cost to get back to the base
            cost = self.path_to_origin_cost(pos=(x, y))

            # Removes tile if no battery left
            if cost > battery:
                self.untried[self.pos].remove(dir)
                continue


    def path_to_origin_cost(self, pos: tuple[int, int]) -> float:
        """
        Calculates the minimun path to the origin, from the current position
        """
        path = find_path(self.graph, pos, (0, 0))   # Method from the Dijkstar library

        if path is None:
            raise Exception("Path not found")
        return path.total_cost


    def move_backtrack(self) -> None:
        """
        Moves to the tile on top of the stack
        """
        # End if there's nowhere else to go
        # if len(self._backtrack[self._pos]) == 0:
        #     # Should never happen
        #     raise Exception("No tiles remaining at untried or backtrack")

        # Move to backtrack tile
        dx, dy = self.backtrack[self.pos].pop(-1).value     # From the latest tile in the Stack

        walk = self.body.walk(dx=dx, dy=dy)

        # Update position
        self.pos = (self.pos[0] + dx, self.pos[1] + dy)
        # if walk < 0:
        #     # Should never happen
        #     if walk == PhysAgent.BUMPED:
        #         raise Exception("Tried to move to an invalid tile at backtracking")
        #     if walk == PhysAgent.TIME_EXCEEDED:
        #         raise Exception("Ran out of battery at backtracking")


    def dfs(self) -> None:
        """
        Moves to the random tile in the untried list
        """
        # Shuffles untired tiles
        random.shuffle(self.untried[self.pos])
        # Move to the first tile in the Stack
        dx, dy = self.untried[self.pos].pop(0).value

        walk = self.body.walk(dx=dx, dy=dy)

        # Update position
        self.pos = (self.pos[0] + dx, self.pos[1] + dy)
        # Add tile to backtrack
        self.backtrack[self.pos].append(Direction((-dx, -dy)))


    def check_victim(self) -> None:
        """
        Checks if there is a victim in the current tile and if the robot has battery to interact with the victim
        """
        # Checks the victim
        victim_id = self.body.check_for_victim()

        # returns -1: if there is no victim at the current position of the agent
        if victim_id >= 0:      
            # Verifies if the robot can interact with the victim
            battery = self.body.rtime - self.COST_READ
            cost = self.path_to_origin_cost(pos=self.pos)
            if cost < battery:
                vital_signs = self.body.read_vital_signals(victim_id)
                self.victims.update(
                    {
                        victim_id: Victim(
                            id=victim_id,
                            pos=self.pos,
                            pSist=vital_signs[1],
                            pDiast=vital_signs[2],
                            qPA=vital_signs[3],
                            pulse=vital_signs[4],
                            resp=vital_signs[5],
                            grav=vital_signs[6],
                            classif=vital_signs[7],
                        )
                    }
                )


    def deliberate(self) -> bool:
        """
        The robot's procees is:
        - Checks his outskirts and add to his map
        - Checks there is a victim in his current tile, and if he can interact with this victim
        - Perform calculations on the current tile, such as:
            - Remove unreachable tiles from untried actions
                - Simulate movement to tile and analyse if he can return after
            - If there are no actions left -> Go to the backtrack tile in the Stack
            - If there are actions left -> DFS
                - Reorder untried tiles randomly and go to the first one of the Stack
        """

        # End condition
        if self.pos == (0, 0) and self.body.rtime < 2 * min(self.COST_LINE, self.COST_DIAG):
            return False

        # Check the outskirts
        self.read_outskirts()

        # Check the victim
        self.check_victim()

        # Remove unreachable tiles from untried actions
        self.remove_invalid_actions()
        # If there are no actions left, then bactrack
        if len(self.untried[self.pos]) == 0:
            self.move_backtrack()
        # Else, continue DFS
        else:
            self.dfs()
        return True