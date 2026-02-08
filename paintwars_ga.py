
from robot import * 
import math
import os
import random

nb_robots = 0
debug = False
LOG_PATH = os.path.join("multiplotCSV", "data", "genetic_log.csv")

class Robot_player(Robot):

    team_name = "Algorithme Génetique"
    robot_id = -1
    iteration = 0

    # GA settings
    MU = 5
    LAMBDA = 20
    MUTATION_RATE = 0.5

    param = []
    bestParam = []
    it_per_evaluation = 400
    trial = 0
    max_essaie = 333

    replay = False
    replay_reset = 1000
    replay_steps = 0

    score = 0.0
    score_total = 0.0
    best_score = -1e18
    best_essaie = -1

    child = []

    # (mu, lambda) population
    population = []
    population_scores = []
    population_index = 0
    generation = 0

    # coverage tracking
    cell_size = 5
    visited_cells = set()

    nb_runs = 1
    run_id = 0

    theta_list = []

    x_0 = 0
    y_0 = 0
    theta_0 = 0 # in [0,360]

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a",evaluations=500,it_per_evaluation=0):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots+=1
        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0
        if not self.population:
            for _ in range(self.LAMBDA):
                self.population.append([random.randint(-1, 1) for _ in range(8)])
            self.population_index = 0
        self.param = self.population[self.population_index][:]
        self.bestParam = self.param[:]

        self.theta_list = [random.randint(0, 359) for _ in range(3)]
        self.it_per_evaluation = it_per_evaluation
        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def reset(self):
        super().reset()
        self.theta = self.theta_list[self.run_id]
        self.visited_cells = set()
        if self.replay:
            self.replay_steps = 0

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # cet exemple montre comment générer au hasard, et évaluer, des stratégies comportementales
        # Remarques:
        # - la liste "param", définie ci-dessus, permet de stocker les paramètres de la fonction de contrôle
        # - la fonction de controle est une combinaison linéaire des senseurs, pondérés par les paramètres (c'est un "Perceptron")

        # toutes les X itérations: le robot est remis à sa position initiale de l'arène avec une orientation aléatoire
        if self.iteration % self.it_per_evaluation == 0:
                if self.iteration > 0 and not self.replay:
                    print("\tRun ", self.run_id+1)
                    print ("\tparameters           =",self.param)
                    print ("\tdistance from origin =",math.sqrt((self.x-self.x_0)**2+(self.y-self.y_0)**2))
                    print("\tTheta                =", self.theta_list[self.run_id])
                    print ("\tScore = ",self.score, "\n")
                    self.score_total = self.score_total + self.score

                    self.run_id = self.run_id + 1

                    if self.run_id < self.nb_runs:
                        self.score = 0.0
                        self.iteration = self.iteration + 1
                        return 0, 0, True # ask for reset

                    # store candidate score
                    self.population_scores.append((self.score_total, self.param[:]))

                    # selection when population is full (mu, lambda)
                    if len(self.population_scores) >= self.LAMBDA:
                        self.population_scores.sort(reverse=True, key=lambda x: x[0])
                        parents = self.population_scores[:self.MU]

                        # update best
                        if parents[0][0] > self.best_score:
                            self.best_score = parents[0][0]
                            self.bestParam = parents[0][1][:]
                            self.best_essaie = self.trial

                        print("Generation", self.generation, "| best score =", self.best_score)
                        self.generation += 1

                        # produce new population (mutation with higher rate)
                        self.population = []
                        for _ in range(self.LAMBDA):
                            base = random.choice(parents)[1][:]
                            for i in range(len(base)):
                                if random.random() < self.MUTATION_RATE:
                                    base[i] = random.choice([-1, 0, 1])
                            self.population.append(base)
                        self.population_scores = []
                        self.population_index = 0
                    else:
                        self.population_index = self.population_index + 1

                    self.param = self.population[self.population_index][:]
                    print("Trying strategy no.", self.trial)

                    self.trial = self.trial + 1
                    
                    if self.trial >= self.max_essaie:
                        self.replay = True
                        self.param = self.bestParam[:]
                        self.replay_steps = 0
                        print("\nSEARCH DONE")
                        print("Best score =", self.best_score)
                        print("Best found at trial =", self.best_essaie)
                        print("POIDS_MUR_GA   =", self.bestParam)
                        print("Replaying best\n")
                    self.score = 0.0
                    self.score_total = 0.0
                    self.run_id = 0
                    self.iteration = self.iteration + 1
                    return 0, 0, True # ask for reset

                if self.replay and self.iteration == 0:
                    self.param = self.bestParam[:]

                if self.iteration == 0 and not self.replay:
                    print("Trying strategy no.", self.trial)

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        translation = math.tanh(
            self.param[0]
            + self.param[1] * sensors[sensor_front_left]
            + self.param[2] * sensors[sensor_front]
            + self.param[3] * sensors[sensor_front_right]
        )
        rotation = math.tanh(
            self.param[4]
            + self.param[5] * sensors[sensor_front_left]
            + self.param[6] * sensors[sensor_front]
            + self.param[7] * sensors[sensor_front_right]
        )

        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)

        # update coverage score
        cx = int(self.x // self.cell_size)
        cy = int(self.y // self.cell_size)
        self.visited_cells.add((cx, cy))
        if not self.replay:
            self.score = len(self.visited_cells)

        if self.replay:
            self.replay_steps = self.replay_steps + 1
            if self.replay_steps >= self.replay_reset:
                self.replay_steps = 0
                self.iteration = self.iteration + 1
                return 0, 0, True

        self.iteration = self.iteration + 1        

        return translation, rotation, False
