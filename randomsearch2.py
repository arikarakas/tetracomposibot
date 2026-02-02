from robot import *
import math
import random

nb_robots = 0
debug = False

class Robot_player(Robot):

    team_name = "RandomSearch2"
    robot_id = -1
    iteration = 0

    param = []
    bestParam = []
    it_per_evaluation = 400

    trial = 0
    max_essaie = 500

    replay = False
    replay_reset = 1000

    score = 0.0
    score_total = 0.0

    best_score = -1e18
    best_essaie = -1

    prev_log_translation = 0.0
    prev_log_rotation = 0.0

    nb_runs = 3
    run_id = 0

    x_0 = 0
    y_0 = 0
    theta_0 = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a", evaluations=500, it_per_evaluation=0):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0

        self.param = [random.randint(-1, 1) for i in range(8)]

        if it_per_evaluation != 0:
            self.it_per_evaluation = it_per_evaluation
        else:
            self.it_per_evaluation = 400

        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def reset(self):
        super().reset()
        self.prev_log_translation = 0.0
        self.prev_log_rotation = 0.0

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        if self.iteration > 0 and (not self.replay):
            vitesseT= self.log_sum_of_translation - self.prev_log_translation 
            vitesseR = self.log_sum_of_rotation - self.prev_log_rotation
            self.score = self.score + vitesseT *(1.0 - abs (vitesseR))

        self.prev_log_translation = self.log_sum_of_translation
        self.prev_log_rotation = self.log_sum_of_rotation

        if self.iteration % self.it_per_evaluation == 0:

            if self.iteration > 0 and (not self.replay):

                self.score_total = self.score_total + self.score

                print("\tparameters =", self.param)
                print("\tscore run =", self.score)
                print("\tscore total =", self.score_total)
                print("\ttranslations =", self.log_sum_of_translation, "; rotations =", self.log_sum_of_rotation)

                self.run_id = self.run_id + 1

                if self.run_id < self.nb_runs:
                    print("\tRun", self.run_id)
                    self.score = 0.0
                    self.prev_log_translation = self.log_sum_of_translation
                    self.prev_log_rotation = self.log_sum_of_rotation
                    self.iteration = self.iteration + 1
                    return 0, 0, True

                if self.score_total > self.best_score:
                    self.best_score = self.score_total
                    self.bestParam = self.param[:]
                    self.best_essaie = self.trial
                    print("NEW BEST at trial", self.best_essaie, "score =", self.best_score, "params =", self.bestParam)

                self.trial = self.trial + 1

                if self.trial >= self.max_essaie:
                    self.replay = True
                    self.param = self.bestParam[:]
                    print("\nSEARCH DONE")
                    print("Best score =", self.best_score)
                    print("Best found at trial =", self.best_essaie)
                    print("Best params =", self.bestParam)
                    print("Replaying best forever\n")
                else:
                    self.param = [random.randint(-1, 1) for i in range(8)]
                    print("Trying strategy no.", self.trial)

                self.score = 0.0
                self.score_total = 0.0
                self.run_id = 0

                self.prev_log_translation = self.log_sum_of_translation
                self.prev_log_rotation = self.log_sum_of_rotation

                self.iteration = self.iteration + 1
                return 0, 0, True

            if self.replay and self.iteration > 0 and (self.iteration % self.replay_reset == 0):
                print("Replay reset")
                self.param = self.bestParam[:]
                self.iteration = self.iteration + 1
                return 0, 0, True

            if self.iteration == 0:
                print("Trying strategy no.", self.trial)

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        translation = math.tanh ( self.param[0] + self.param[1] * sensors[sensor_front_left] + self.param[2] * sensors[sensor_front] + self.param[3] * sensors[sensor_front_right] )
        rotation = math.tanh ( self.param[4] + self.param[5] * sensors[sensor_front_left] + self.param[6] * sensors[sensor_front] + self.param[7] * sensors[sensor_front_right] )

        self.iteration = self.iteration + 1
        return translation, rotation, False