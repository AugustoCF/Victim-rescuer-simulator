from rescuer import Rescuer
from fuzzy import Fuzzy
from sklearn.cluster import KMeans
import numpy as np
import os
from matplotlib import pyplot as plt
from dijkstar import Graph, find_path
from aux_file import DIRECTIONS, Direction
from generic_alg import GeneticAlgorithmEngine
from physical_agent import PhysAgent
from victim import Victim


class RescuerBoss(Rescuer):
    def __init__(self, env, config_file, rescuer_list, fuzzy):
        super().__init__(env, config_file)

        self.fuzzy = fuzzy
        self.inactive_exp_counter = 0
        self.map_victims = {}
        self.map_obstacles = {}
        self.map_graph = Graph()
        self.victims_graph = Graph()
        self.rescuers = rescuer_list
        self.victim_clusters : list[list[Victim]] = [None,None,None,None]

        self.action_cost = {
            "N": 1, "S": 1, "E": 1, "W": 1,
            "NE": 1.5, "SE": 1.5, "NW": 1.5, "SW": 1.5
        }

    def alert_explorer_inactive(self) -> None:
        self.inactive_exp_counter += 1

        if (self.inactive_exp_counter == 1):
            self.prepare_rescuers()

    def prepare_rescuers(self) -> None:
        for chave, valor in self.map_victims.items():
           valor.classif = self.fuzzy.predict_fuzzy(valor.pulse, valor.qPA, valor.resp)
        
        self.cluster_victims()
        self.create_map_graph()
        self.create_victims_graph()
        self._run_rescue()

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
        for chave,valor in clusters.items():
            self.victim_clusters[chave-1] = valor
          
    def create_map_graph(self) -> None:
        for coordenada, tile_type in self.map_obstacles.items():
            if tile_type == 0:
                self.map_graph.add_node(coordenada)
                for dir in DIRECTIONS:
                    next_coord = (coordenada[0] + dir.value[0], coordenada[1] + dir.value[1])
                    if next_coord in self.map_obstacles and self.map_obstacles[next_coord] == 0:
                        self.map_graph.add_edge(coordenada, next_coord,self.action_cost[dir.name])  # A -> B
                        self.map_graph.add_edge(next_coord, coordenada, self.action_cost[dir.name])  # B -> A
                        
    def create_victims_graph(self) -> None:
        self.victims_graph.add_node(-1)

        for id, victim in self.map_victims.items():
            self.victims_graph.add_node(id)
            for id2, victim2 in self.map_victims.items():
                cost = find_path(self.map_graph, victim.pos, victim2.pos).total_cost
                self.victims_graph.add_edge(id, id2, cost)
                self.victims_graph.add_edge(id2, id, cost)
            
            cost_origin = find_path(self.map_graph, victim.pos, (0,0)).total_cost
            self.victims_graph.add_edge(-1, id, cost_origin)
            self.victims_graph.add_edge(id, -1, cost_origin)

    def _calculate_directions(self,cluster_index: int) -> list[Direction]:
        
        victim_list = self.victim_clusters[cluster_index]

        ga = GeneticAlgorithmEngine(
            victims_dict= self.map_victims,
            COST_RESCUE=self.COST_FIRST_AID,
            TIME_LIMIT=self.TLIM,
            POPULATION_SIZE=100,
            ELITE_SIZE=10,
            MUTATION_RATE=0.1,
            GENERATION_LIMIT=100,
            EARLY_STOP_GENERATIONS=10,
            VICTIMS_GRAPH = self.victims_graph,
            MAP_GRAPH = self.map_graph,
        )

        actions = ga.run()

        return actions
    
    def _run_rescue(self):
        for i, rescuer in enumerate(self.rescuers):
            self.rescuers[i].directions = self._calculate_directions(i)
            self.rescuers[i].victims = {victim.id: victim for victim in self.victim_clusters[i]}
            self.rescuers[i].body.set_state(PhysAgent.ACTIVE)
        
        # Erase victim result file
        open("salvas.txt", "w").close()
        
        
