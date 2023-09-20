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