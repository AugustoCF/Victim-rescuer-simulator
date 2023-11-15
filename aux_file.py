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

priority_directions = [['N', 'E', 'S', 'W', 'NE', 'SE', 'NW', 'SW'],
                      ['S', 'E', 'N', 'W', 'SW', 'NW', 'SE', 'NE'],
                      ['W', 'N', 'E', 'S', 'SE', 'SW', 'NE', 'NW'],
                      ['W', 'S', 'E', 'N', 'NW', 'NE', 'SW', 'SE']
]


