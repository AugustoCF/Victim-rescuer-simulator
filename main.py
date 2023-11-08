import os
import sys
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from aux_file import priority_directions

## importa classes
from environment import Env
from explorer_robot import ExplorerRobot
from rescuer import Rescuer


def main(data_folder_name):
    # Set the path to config files and data files for the environment
    current_folder = os.path.abspath(os.getcwd())
    data_folder = os.path.abspath(os.path.join(current_folder, data_folder_name))

    all_victims = {}

    # Instantiate the environment
    env = Env(data_folder)

    # config files for the agents
    rescuer_file = os.path.join(data_folder, "rescuer_config.txt")

    explorer_file_1 = os.path.join(data_folder, "explorer_config_q1.txt")
    explorer_file_2 = os.path.join(data_folder, "explorer_config_q2.txt")
    explorer_file_3 = os.path.join(data_folder, "explorer_config_q3.txt")
    explorer_file_4 = os.path.join(data_folder, "explorer_config_q4.txt")

    # Instantiate agents rescuer and explorer
    resc = Rescuer(env, rescuer_file)

    # Explorer needs to know rescuer to send the map
    # that's why rescuer is instatiated before
    exp1 = ExplorerRobot(env, explorer_file_1, priority_directions[0])
    exp2 = ExplorerRobot(env, explorer_file_2, priority_directions[1])
    exp3 = ExplorerRobot(env, explorer_file_3, priority_directions[2])
    exp4 = ExplorerRobot(env, explorer_file_4, priority_directions[3])

    # Run the environment simulator
    env.run()

    #print(exp1.NAME)
    #for id, data in exp1.victims.items():
    #        print(f"Id: {id}, Pos: {data.pos}, pulse: {data.pulse}, resp: {data.resp}, qPA: {data.qPA}")
    
    for id, data in exp1.victims.items():
        all_victims[id] = data

    for id, data in exp2.victims.items():
        all_victims[id] = data

    for id, data in exp3.victims.items():
        all_victims[id] = data

    for id, data in exp4.victims.items():
        all_victims[id] = data
    
    for id, data in all_victims.items():
        print(f"id: {id}, posicao x: {data.pos[0]} y: {data.pos[1]}")
    

def classify_victims (self) -> None:
    pulse = ctrl.Antecedent(np.arange(0, 201, 1), 'Pulse')
    qPA = ctrl.Antecedent(np.arange(-10, 11, 1), 'qPA')
    resp = ctrl.Antecedent(np.arange(0, 23, 1), 'Resp')
    classification = ctrl.Consequent(np.arange(1, 5, 1), 'Classification')

    pulse['Low'] = fuzz.trimf(pulse.universe, [0, 1, 80])
    pulse['Mid'] = fuzz.trimf(pulse.universe, [60, 80, 130])
    pulse['High'] = fuzz.trimf(pulse.universe, [110, 200, 200])

    resp['Low'] = fuzz.trimf(pulse.universe, [0, 0, 12])
    resp['Mid'] = fuzz.trimf(pulse.universe, [10, 15, 21])
    resp['High'] = fuzz.trimf(pulse.universe, [20, 22, 22])

    qPA['Min-'] = fuzz.trimf(pulse.universe, [-10, -10, -2])
    qPA['Min+'] = fuzz.trimf(pulse.universe, [2, 10, 10])
    qPA['High'] = fuzz.trimf(pulse.universe, [-3, 0, 3])

    classification['Crítico'] = fuzz.trimf(classification.universe, [1, 1, 2])
    classification['Instável'] = fuzz.trimf(classification.universe, [1, 2, 3])
    classification['Pot Estável'] = fuzz.trimf(classification.universe, [2, 3, 4])
    classification['Estável'] = fuzz.trimf(classification.universe, [3, 4, 4])
    
    

if __name__ == '__main__':
    """ To get data from a different folder than the default called data
    pass it by the argument line"""

    if len(sys.argv) > 1:
        data_folder_name = sys.argv[1]
    else:
        # data_folder_name = os.path.join("datasets", "data_100x80_132vic")
        data_folder_name = os.path.join("datasets", "data_20x20_42vic")

    main(data_folder_name)
