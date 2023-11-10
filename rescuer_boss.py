from rescuer import Rescuer
from fuzzy import Fuzzy
from sklearn.cluster import KMeans
import numpy as np
from matplotlib import pyplot as plt

class RescuerBoss(Rescuer):
    def __init__(self, env, config_file, rescuer_list, fuzzy):
        super().__init__(env, config_file)

        self.fuzzy = fuzzy
        self.inactive_exp_counter = 0
        self.map_victims = {}
        self.map_obstacles = {}

    def alert_explorer_inactive(self) -> None:
        self.inactive_exp_counter += 1

        if (self.inactive_exp_counter == 1):
            self.prepare_rescuers()
       



    def prepare_rescuers(self) -> None:
        for chave, valor in self.map_victims.items():
           valor.classif = self.fuzzy.predict_fuzzy(valor.pulse, valor.qPA, valor.resp)
        
        self.cluster_victims()
        


    def cluster_victims(self) -> None:
        data = []
        coisas = {}

        for chave, valor in self.map_victims.items():
            d = [valor.pos[0], valor.pos[1], valor.classif]
            data.append(np.array(d))
            coisas[tuple(d)] = valor

        data = np.array(data)

        km = KMeans(n_clusters=4)
        y_km = km.fit_predict(data)
        clusters={1:[],2:[],3:[],4:[]}
        for i in range(len(data)):
            clusters[y_km[i]+1].append(coisas[tuple(data[i].tolist())])
        
        for chave, valor in clusters.items():
            with open(f'cluster{chave}.txt', 'w') as outfile:
                for i in valor:
                    outfile.write(f"{i.id},{i.pos[0]},{i.pos[1]},{0},{i.classif}\n")
        # for i in range(len(km.cluster_centers_)):

            

        
