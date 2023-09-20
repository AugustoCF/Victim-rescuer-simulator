from explorer_robot import Explorer_robot


class Explorer_robot_Q1(Explorer_robot):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

    def _bias_xdir(self, delta: float) -> bool:
        return delta > 0

    def _bias_ydir(self, delta: float) -> bool:
        return delta < 0


class Explorer_robot_Q2(Explorer_robot):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

    def _bias_xdir(self, delta: float) -> bool:
        return delta > 0

    def _bias_ydir(self, delta: float) -> bool:
        return delta > 0


class Explorer_robot_Q3(Explorer_robot):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

    def _bias_xdir(self, delta: float) -> bool:
        return delta < 0

    def _bias_ydir(self, delta: float) -> bool:
        return delta > 0


class Explorer_robot_Q4(Explorer_robot):
    def __init__(self, env, config_file):
        super().__init__(env, config_file)

    def _bias_xdir(self, delta: float) -> bool:
        return delta < 0

    def _bias_ydir(self, delta: float) -> bool:
        return delta < 0
