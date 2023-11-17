import random
from victim import Victim
from dijkstar import Graph, find_path
from aux_file import Direction, DIRECTIONS


class Location:
    def __init__(self, id: int, x: int, y: int, classif: int):
        self.id = id
        self.x = x
        self.y = y
        self.severity = classif
        
class Individual:
    """
    Individual representing a sequence of locations in the map.
    Contains a single chromosome.
    """
    def __init__(self, genes: list[Location]):
        self.genes = genes
        
class GeneticAlgorithmEngine:
    def __init__(
        self,
        victims_dict: {},
        COST_RESCUE: float,
        TIME_LIMIT: float,
        POPULATION_SIZE: int,
        ELITE_SIZE: int,
        MUTATION_RATE: float,
        GENERATION_LIMIT: int,
        EARLY_STOP_GENERATIONS: int,
        VICTIMS_GRAPH: Graph,
        MAP_GRAPH: Graph,
    ):
        
        locations: list[tuple[int, int, int, int]] = [(-1, 0, 0, -1)]
        for chave, valor in victims_dict.items():
            locations.append((valor.id, valor.pos[0], valor.pos[1], valor.classif))
        # array of victims only
        victims = locations[1:]
        
        
        self._GENES: list[Location] = []
        for v in victims:
            v_id, v_x, v_y, v_classif = v[0], v[1], v[2], v[3]
            self._GENES.append(Location(v_id, v_x, v_y, v_classif))
        # initialize population pool
        self._population: list[Individual] = []
        # initialize variables
        # self._COST_LINE = 1
        # self._COST_DIAG = 1.5
        self._COST_RESCUE = COST_RESCUE
        self._POPULATION_SIZE = POPULATION_SIZE
        self._ELITE_SIZE = ELITE_SIZE
        self._MUTATION_RATE = MUTATION_RATE
        self._GENERATION_LIMIT = GENERATION_LIMIT
        self._EARLY_STOP_GENERATIONS = EARLY_STOP_GENERATIONS
        self._TIME_LIMIT = TIME_LIMIT
        self._TOTAL_SEVERITY = sum([v[3] for v in victims])
        self._current_best_individual: Individual = None
        self._current_best_fitness: float = float("-inf")
        self._current_best_is_valid = False  # check for time limit exceeded   
        self._victims_graph = VICTIMS_GRAPH
        self._map_graph = MAP_GRAPH
        
    def _fitness(self, individual: Individual) -> tuple[float, float]:
        total_cost = 0
        accumulated_severity = 0
        
        current_location = Location(-1,0,0,-1)
        
        for location in individual.genes:
            cost = self._victims_graph.get_edge(current_location.id, location.id)
            total_cost += cost + self._COST_RESCUE
            accumulated_severity += location.severity
            current_location = location
        cost = self._victims_graph.get_edge(current_location.id, -1)
        total_cost += cost
        
        cost_fitness = 1 - total_cost / self._TIME_LIMIT
        severity_fitness = accumulated_severity / self._TOTAL_SEVERITY
        
        return cost_fitness, severity_fitness
        
    def _rank_population(self) -> list[tuple[float, int]]:
        """
        Rank the population based on fitness. The best fitness will be saved.
        The returned list is a list of tuples (fitness, individual index).
        """
        ranked_population = []
        for i in range(len(self._population)):
            individual = self._population[i]
            # calculate fitness for each individual
            cost_fit, severity_fit = self._fitness(individual)
            fitness = cost_fit + severity_fit
            ranked_population.append((fitness, i))
            # remember best individual
            if fitness > self._current_best_fitness:
                self._current_best_fitness = fitness
                self._current_best_individual = individual
                self._current_best_is_valid = cost_fit >= 0
        # sort population by fitness
        ranked_population.sort(key=lambda x: x[0], reverse=True)
        return ranked_population

    def _roulette_wheel_selection(
            self, ranked_population: list[tuple[float, int]]
        ) -> list[Individual]:
            """
            Select individuals for the next generation based on their fitness.
            The best individuals will be selected more often.
            """
            selection_results = []
            for i in range(self._ELITE_SIZE):
                selection_results.append(self._population[ranked_population[i][1]])
            fitness_sum = sum([ranked_population[i][0] for i in range(len(ranked_population))])
            for i in range(self._ELITE_SIZE, len(self._population)):
                r = random.uniform(0, fitness_sum)
                current_sum = 0
                for j in range(len(ranked_population)):
                    current_sum += ranked_population[j][0]
                    if current_sum > r:
                        selection_results.append(self._population[ranked_population[j][1]])
                        break
            return selection_results
            
    def _breed_parents(self, parent1: Individual, parent2: Individual) -> Individual:
        """
        Create a child from two parents by combining their genes.
        """
        # select a random sequence of genes from parent1
        gene_a = int(random.random() * len(parent1.genes))
        gene_b = int(random.random() * len(parent1.genes))
        start_gene = min(gene_a, gene_b)
        end_gene = max(gene_a, gene_b)

        # create first half of the child with the selected genes from parent1
        child_1 = []
        for i in range(start_gene, end_gene):
            child_1.append(parent1.genes[i])
        # create second half of the child with the remaining genes from parent2
        child_2 = [gene for gene in parent2.genes if gene not in child_1]

        # combine the two halves
        child = child_1 + child_2
        child_individual = Individual(child)
        return child_individual
                
    def _generate_children(self, selected_population: list[Individual]) -> list[Individual]:
        """
        Create children by breeding pairs of parents.
        """
        children = selected_population[: self._ELITE_SIZE].copy()
        for i in range(self._ELITE_SIZE, len(selected_population)):
            child = self._breed_parents(selected_population[i], random.choice(selected_population))
            children.append(child)
        return children
        
    def _mutate_children(self, children: list[Individual]) -> list[Individual]:
        """
        Mutate children by swapping two genes.
        """
        mutated_children = children.copy()
        for i in range(self._ELITE_SIZE, len(mutated_children)):
            if random.random() < self._MUTATION_RATE:
                gene_a = int(random.random() * len(mutated_children[i].genes))
                gene_b = int(random.random() * len(mutated_children[i].genes))

                mutated_children[i].genes[gene_a], mutated_children[i].genes[gene_b] = (
                    mutated_children[i].genes[gene_b],
                    mutated_children[i].genes[gene_a],
                )
        return mutated_children
        
    def _run_generation(self):
        """
        Run a single iteration of the evolution process.
        """
        # rank the current population
        ranked_population = self._rank_population()
        # select individuals for the next generation
        selected_population = self._roulette_wheel_selection(ranked_population)
        # create children
        children = self._generate_children(selected_population)
        # mutate children
        mutated_children = self._mutate_children(children)
        # update population
        self._population = mutated_children
            
    def _calculate_path(self, individual: Individual) -> list[Direction]:
        """
        Calculate the path to follow based on the given individual.
        """
        total_path: list[Direction] = []
        origin = (0, 0)
        current_location = origin
        for location in individual.genes:
            path = find_path(self._map_graph, current_location, (location.x, location.y))
            # iterate through the path and add the directions to the total path
            for i in range(len(path.nodes) - 1):
                current_node = path.nodes[i]
                next_node = path.nodes[i + 1]
                dx = next_node[0] - current_node[0]
                dy = next_node[1] - current_node[1]
                direction = Direction((dx, dy))
                total_path.append(direction)
            current_location = (location.x, location.y)
        # from last victim to origin
        path = find_path(self._map_graph, current_location, origin)
        # iterate through the path and add the directions to the total path
        for i in range(len(path.nodes) - 1):
            current_node = path.nodes[i]
            next_node = path.nodes[i + 1]
            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]
            direction = Direction((dx, dy))
            total_path.append(direction)
        return total_path
        
    def run(self) -> list[Direction]:
        """
        Run the genetic algorithm and return a list of directions to follow.
        """
        print("Running genetic algorithm...")
        # try to find a valid solution for reducing chromosome lengths
        run_finished = False
        for chromosome_len in range(len(self._GENES), 0, -1):
            print(f"Trying chromosome length: {chromosome_len}")
            if run_finished:
                break
            # generate initial population
            chromosomes = [
                random.sample(self._GENES, chromosome_len) for _ in range(self._POPULATION_SIZE)
            ]
            self._population = [Individual(chromosome) for chromosome in chromosomes]
            # run evolution
            repeated_fitness_count = 0
            best_fitness = self._current_best_fitness
            for generation in range(self._GENERATION_LIMIT):
                # run next generation
                self._run_generation()
                # print best individual
                victim_ids = ""
                for v in self._current_best_individual.genes:
                    victim_ids += f"({v.x},{v.y})" + "->"
                victim_ids = victim_ids[:-2]
                print(f"GENERATION {generation}\tBEST FITNESS - {self._current_best_fitness}")
                # print("Best route: " + victim_ids)
                # check if the fitness has not improved for EARLY_STOP_GENERATIONS generations
                if self._current_best_fitness <= best_fitness:
                    repeated_fitness_count += 1
                else:
                    repeated_fitness_count = 0
                    best_fitness = self._current_best_fitness
                if repeated_fitness_count >= self._EARLY_STOP_GENERATIONS:
                    # if the best is valid, stop and return the best individual
                    # otherwise, reduce the chromosome length and try again
                    if self._current_best_is_valid:
                        run_finished = True
                    break
            else:
                # if the best is valid, stop and return the best individual
                # otherwise, reduce the chromosome length and try again
                if self._current_best_is_valid:
                    run_finished = True
        else:
            # could not find a valid solution
            print("Could not find a valid solution")

        print("Finished running genetic algorithm")
        victims = ""
        for v in self._current_best_individual.genes:
            victims += f"({v.x},{v.y})" + "->"
        victims = victims[:-2]
        print(f"Best fitness: {self._current_best_fitness}")
        print("Best route: " + victims)
        return self._calculate_path(self._current_best_individual)