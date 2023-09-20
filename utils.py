from typing import Any
from enum import Enum


class Direction(Enum):
    """
    Directions of the tiles
    """
    N = (0, -1)
    NE = (1, -1)
    E = (1, 0)
    SE = (1, 1)
    S = (0, 1)
    SW = (-1, 1)
    W = (-1, 0)
    NW = (-1, -1)
    NONE = (0, 0)

DIRECTIONS = [
    Direction.N,
    Direction.NE,
    Direction.E,
    Direction.SE,
    Direction.S,
    Direction.SW,
    Direction.W,
    Direction.NW,
]
"""
The order of directions that is returned to the agent (N -> NE -> E -> ...).
"""


LINE_DIRECTIONS = set([Direction.N, Direction.E, Direction.S, Direction.W])
"""
Straight line directions.
"""

DIAG_DIRECTIONS = set([Direction.NE, Direction.SE, Direction.SW, Direction.NW])
"""
Diagonal line directions.
"""

class Victim:
    """
    Class for the victims
    """
    def __init__(
        self,
        id: int,
        pos: tuple[int, int],
        pSist: float,
        pDiast: float,
        qPA: float,
        pulse: float,
        resp: float,
        grav: int,
        classif: int
    ):
        self.id = id
        self.pos = pos


class VitalSigns:
    """
    VitalSigns used in the class Victim.
    """
    pSist = 0.0
    pDiast = 0.0
    qPA = 0.0
    pulso = 0.0
    resp = 0.0
    gravidade = 0.0
    classe = 0

    def __init__(self, pSist: float, pDiast: float, qPA: float, pulso: float, resp: float, ):
        self.pSist = pSist
        self.pDiast = pDiast
        self.qPA = qPA
        self.pulso = pulso
        self.resp = resp
