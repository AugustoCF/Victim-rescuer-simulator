import os
import random
import numpy as np

from abstract_agent import AbstractAgent
from physical_agent import PhysAgent
from aux_file import Direction
from victim import Victim

class RescuerRobot(AbstractAgent):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

        # The calculated list of directions it has to move to
        self.directions: list[Direction] = []

        # The list of victims it has to rescue
        self.victims : dict[int,Victim] = {}

        # Victims already rescued
        self.victims_rescued = set()

        # Start deactivated
        self.body.set_state(PhysAgent.IDLE)

    def deliberate(self) -> bool:
        """
        Executes its predefined sequence of actions
        """

        # If there are no more actions to execute, stop
        if len(self.directions) == 0:
            return False

        # move to next tile
        next_move = self.directions.pop(0)
        dx, dy = next_move.value
        self.body.walk(dx=dx, dy=dy)
        
        # rescue victim if possible
        victim_id = self.body.check_for_victim()
        if (victim_id != -1) and (victim_id in self.victims) and (victim_id not in self.victims_rescued):
            self.body.first_aid(victim_id)

            self.victims_rescued.add(victim_id)
            # log the rescue
            print(
                f"{self.NAME}: rescued {victim_id} at ({self.body.x},{self.body.y})"
            )
            with open("salvas.txt", "a") as f:
                v = self.victims[victim_id]
                f.write(f"{victim_id},{self.body.x},{self.body.y},{v.grav},{v.classif}\n")

        return True
