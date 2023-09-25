import numpy as np
import random
from dijkstar import Graph, find_path
from abc import abstractmethod

from abstract_agent import AbstractAgent
from physical_agent import PhysAgent
from rescuer import Rescuer
from victim import Victim

from rescuer import Rescuer
from aux_file import Direction, DIRECTIONS

class Explorer_robot (AbstractAgent):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

        self.graph = Graph()                                            
        self.pos = (0,0)
        self.victims = {}                            
        self.tile_tested = {}                       
        self.path_not_tested = {}       
        self.backtrack = {}

        self.action_cost = {
            "N": 1, "S": 1, "E": 1, "W": 1,
            "NE": 1.5, "SE": 1.5, "NW": 1.5, "SW": 1.5
        }

        self.add_position_to_map(dir= Direction.NONE, tile_type= PhysAgent.CLEAR)  
        self.read_nearby_tiles()
                                      

    def position_shift (self, dir: Direction, x, y):
        dx, dy = dir.value
        return x + dx, y + dy


    def oppositeDirection(self, dx, dy):
        if   (dx, dy) == Direction.N.value : return Direction.S
        elif (dx, dy) == Direction.S.value : return Direction.N
        elif (dx, dy) == Direction.E.value : return Direction.W
        elif (dx, dy) == Direction.W.value : return Direction.E
        elif (dx, dy) == Direction.NE.value : return Direction.SW
        elif (dx, dy) == Direction.SE.value : return Direction.NW
        elif (dx, dy) == Direction.NW.value : return Direction.SE
        elif (dx, dy) == Direction.SW.value : return Direction.NE


    def add_position_to_map(self, dir: Direction, tile_type: int) -> None:
        x, y = self.position_shift(dir, self.pos[0], self.pos[1])

        if (x, y) in self.tile_tested:
            return

        self.tile_tested[(x, y)] = tile_type
        self.path_not_tested[(x, y)] = DIRECTIONS.copy()
        self.backtrack[(x, y)] = []

        for dir in DIRECTIONS:
            x2, y2 = self.position_shift(dir, x, y)
            if ((x2, y2) not in self.tile_tested or self.tile_tested[(x2, y2)] != PhysAgent.CLEAR):
                continue

            self.graph.add_edge((x, y), (x2, y2), self.action_cost[dir.name])   # A -> B
            self.graph.add_edge((x2, y2), (x, y), self.action_cost[dir.name])   # B -> A
            

    def read_nearby_tiles(self) -> None:
        nearby_tiles = self.body.check_obstacles()     

        for i in range(8):
            self.add_position_to_map(dir=DIRECTIONS[i], tile_type=nearby_tiles[i])


    def remove_unreachable_tiles(self) -> None:
        for dir in self.path_not_tested[self.pos].copy():       
            x, y = self.position_shift(dir, self.pos[0], self.pos[1])

            tile_type = self.tile_tested[(x, y)]
            if tile_type == PhysAgent.WALL or tile_type == PhysAgent.END:
                self.path_not_tested[self.pos].remove(dir)      
                continue

            walk_cost = self.action_cost[dir.name]
            time_left = self.body.rtime - walk_cost

            cost = self.path_to_origin(pos=(x, y))

            if cost > time_left:
                self.path_not_tested[self.pos].remove(dir)
                continue


    def path_to_origin(self, pos: tuple[int, int]) -> float:
        path = find_path(self.graph, pos, (0, 0))   # Method from the Dijkstar library

        if path is None:
            raise ValueError("Path not found")
        return path.total_cost


    def move_Backtrack(self) -> None:
        dx, dy = self.backtrack[self.pos].pop(-1).value     

        walk = self.body.walk(dx=dx, dy=dy)

        self.pos = (self.pos[0] + dx, self.pos[1] + dy)


    def move_DFS(self) -> None:
        random.shuffle(self.path_not_tested[self.pos])
  
        dx, dy = self.path_not_tested[self.pos].pop(0).value

        walk = self.body.walk(dx=dx, dy=dy)

        self.pos = (self.pos[0] + dx, self.pos[1] + dy)
    
        self.backtrack[self.pos].append(self.oppositeDirection(dx, dy))


    def check_for_victim(self) -> None:
        victim_id = self.body.check_for_victim()

        if victim_id != -1:      
            time_left = self.body.rtime - self.COST_READ
            cost = self.path_to_origin(pos=self.pos)

            if cost < time_left:
                vital_signs = self.body.read_vital_signals(victim_id)
                self.victims.update
                ({
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
                        )})


    def deliberate(self) -> bool:
        if self.pos == (0, 0) and self.body.rtime < 2 * min(self.COST_LINE, self.COST_DIAG):
            return False
        

        self.read_nearby_tiles()
        self.check_for_victim()
        self.remove_unreachable_tiles()
  
        if len(self.path_not_tested[self.pos]) == 0:
            self.move_Backtrack()
        else:
            self.move_DFS()
        return True