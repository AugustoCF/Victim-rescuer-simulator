import os
import sys
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from aux_file import priority_directions
from rescuer_robot import RescuerRobot
## importa classes
from environment import Env
from explorer_robot import ExplorerRobot
from rescuer import Rescuer
from rescuer_boss import RescuerBoss
from fuzzy import Fuzzy


def main(data_folder_name):
    # Set the path to config files and data files for the environment
    current_folder = os.path.abspath(os.getcwd())
    data_folder = os.path.abspath(os.path.join(current_folder, data_folder_name))

    all_victims = {}

    # Instantiate the environment
    env = Env(data_folder)

    # config files for the agents
    rescuer_file = os.path.join(data_folder, "rescuer_config.txt")

    explorer_file_1 = os.path.join(data_folder, "explorer_config.txt")
    # explorer_file_2 = os.path.join(data_folder, "explorer_config_q2.txt")
    # explorer_file_3 = os.path.join(data_folder, "explorer_config_q3.txt")
    # explorer_file_4 = os.path.join(data_folder, "explorer_config_q4.txt")

    resc1 = RescuerRobot(env, rescuer_file)
    resc2 = RescuerRobot(env, rescuer_file)
    resc3 = RescuerRobot(env, rescuer_file)

    training_file = os.path.join("datasets", "data_800vic", "sinais_vitais_com_label.txt")

    fuzzy = Fuzzy()
    fuzzy.training_fuzzy(training_file)
    # Instantiate agents rescuer and explorer
    resc_b = RescuerBoss(env, rescuer_file, [resc1, resc2, resc3], fuzzy)

    # Explorer needs to know rescuer to send the map
    # that's why rescuer is instatiated before
    exp1 = ExplorerRobot(env, explorer_file_1, priority_directions[0], resc_b)
    exp2 = ExplorerRobot(env, explorer_file_1, priority_directions[1], resc_b)
    exp3 = ExplorerRobot(env, explorer_file_1, priority_directions[2], resc_b)
    exp4 = ExplorerRobot(env, explorer_file_1, priority_directions[3], resc_b)



    # Run the environment simulator
    env.run()

    

if __name__ == '__main__':
    """ To get data from a different folder than the default called data
    pass it by the argument line"""

    if len(sys.argv) > 1:
        data_folder_name = sys.argv[1]
    else:
        # data_folder_name = os.path.join("datasets", "data_100x80_132vic")
        data_folder_name = os.path.join("datasets", "data_100x80_225vic")

    main(data_folder_name)
