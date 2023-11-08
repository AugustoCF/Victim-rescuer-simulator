from typing import Tuple


class Victim:
    """
    Class for the victims
    """
    def __init__(
        self,
        id: int,
        pos: Tuple[int, int],
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
        self.pSist = pSist
        self.pDiast = pDiast
        self.qPA = qPA
        self.pulse = pulse
        self.resp = resp
        self.grav = grav
        self.classif = classif