
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

    prev_translation = 0.0
    prev_rotation = 0.0

    nb_runs = 3
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
        self.param = [random.randint(-1, 1) for i in range(8)]
        self.bestParam = self.param[:]

        self.theta_list = [random.randint(0, 359) for _ in range(3)]
        self.it_per_evaluation = it_per_evaluation
        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def reset(self):
        super().reset()
        self.theta = self.theta_list[self.run_id]
        self.prev_translation = 0.0
        self.prev_rotation = 0.0
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
                    print ("\ttranslations         =",self.log_sum_of_translation,"; rotations =",self.log_sum_of_rotation) # *effective* translation/rotation (ie. measured from displacement)
                    print ("\tdistance from origin =",math.sqrt((self.x-self.x_0)**2+(self.y-self.y_0)**2))
                    print("\tTheta                =", self.theta_list[self.run_id])
                    print ("\tScore = ",self.score, "\n")
                    self.score_total = self.score_total + self.score

                    self.run_id = self.run_id + 1

                    if self.run_id < self.nb_runs:
                        self.score = 0.0
                        self.prev_translation = self.log_sum_of_translation
                        self.prev_rotation = self.log_sum_of_rotation
                        self.iteration = self.iteration + 1
                        return 0, 0, True # ask for reset

                    accepted = self.score_total > self.best_score
                    if accepted:
                        self.best_score = self.score_total
                        candidate = self.child if self.child else self.param
                        self.bestParam = candidate[:]
                        self.best_essaie = self.trial
                    else:
                        self.param = self.bestParam[:]

                    print("Generation", self.trial, "| child score =", self.score_total, "| parent score =", self.best_score, "|", "ACCEPT" if accepted else "REJECT")
                    line = f"{self.trial},{self.score_total},{self.best_score}"
                    print(line)
                    with open(LOG_PATH, "a") as f:
                        f.write(line + "\n")

                    if accepted:
                        print("NEW BEST at trial", self.best_essaie, "score =", self.best_score, "params =", self.bestParam)

                    self.trial = self.trial + 1
                    
                    if self.trial >= self.max_essaie:
                        self.replay = True
                        self.param = self.bestParam[:]
                        self.replay_steps = 0
                        print("\nSEARCH DONE")
                        print("Best score =", self.best_score)
                        print("Best found at trial =", self.best_essaie)
                        print("Best params =", self.bestParam)
                        print("Replaying best\n")
                    else:
                        self.child = self.bestParam[:]
                        randindex = random.randint(0,7)
                        choices = [-1, 0, 1]
                        choices.remove(self.child[randindex])
                        self.child[randindex] = random.choice(choices)
                        self.param = self.child
                        print("Trying strategy no.", self.trial)

                    self.score = 0.0
                    self.score_total = 0.0
                    self.run_id = 0
                    self.prev_translation = self.log_sum_of_translation
                    self.prev_rotation = self.log_sum_of_rotation
                    self.iteration = self.iteration + 1
                    return 0, 0, True # ask for reset

                if self.replay and self.iteration == 0:
                    self.param = self.bestParam[:]

                if self.iteration == 0 and not self.replay:
                    print("Trying strategy no.", self.trial)

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        translation = math.tanh ( self.param[0] + self.param[1] * sensors[sensor_front_left] + self.param[2] * sensors[sensor_front] + self.param[3] * sensors[sensor_front_right] )
        rotation = math.tanh ( self.param[4] + self.param[5] * sensors[sensor_front_left] + self.param[6] * sensors[sensor_front] + self.param[7] * sensors[sensor_front_right] )

        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)

        vitesseT = self.log_sum_of_translation - self.prev_translation
        vitesseR = self.log_sum_of_rotation - self.prev_rotation
        if not self.replay:
            self.score = self.score + vitesseT * (1.0 - abs(vitesseR))
        self.prev_translation = self.log_sum_of_translation
        self.prev_rotation = self.log_sum_of_rotation

        if self.replay:
            self.replay_steps = self.replay_steps + 1
            if self.replay_steps >= self.replay_reset:
                self.replay_steps = 0
                self.iteration = self.iteration + 1
                return 0, 0, True

        self.iteration = self.iteration + 1        

        return translation, rotation, False
