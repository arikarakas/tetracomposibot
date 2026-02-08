# Projet "robotique" IA&Jeux 2025
#
# Binome:
#  Prénom Nom No_étudiant/e : Ari Karakas 21313611
#  Prénom Nom No_étudiant/e : Amrani Leiticia 21322966
#
# check robot.py for sensor naming convention
# all sensor and motor value are normalized (from 0.0 to 1.0 for sensors, -1.0 to +1.0 for motors)

from robot import *
import random

nb_robots = 0

class Robot_player(Robot):

    team_name = "DRPN"

    robot_id = -1
    memory = 0  # un seul entier autorisé (obligatoire)

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0, name="Robot " + str(self.robot_id), team=self.team_name)

    def borne(self, x, a=-1.0, b=1.0):
        """Sature x dans [a,b]."""
        if x < a:
            return a
        if x > b:
            return b
        return x
    
    def lire_memoire(self, mem):
        """
        mode:
          0 = normal
          1 = escape (mur imminent)
          2 = hunt (ennemi détecté)
        timer:
          compteur (durée restante du mode)
        """
        timer = mem % 100
        mode = (mem // 100) % 10
        last_dist = mem // 1000 / 1000.0
        return mode, timer, last_dist

    def ecrire_memoire(self, mode, timer, dist):
        """Encode (mode,timer) dans un entier."""
        return int(int(round(dist * 1000)) * 1000 + (mode % 10) * 100 + min(99, max(0, timer)))
    
    def braitenberg_rotation(self, distances, poids):
        """
        Rotation Braitenberg:
          rotation = sum(poids[i] * (1 - distances[i]))
        => plus c'est proche, plus ça influence.
        """
        r = 0.0
        for i in range(8):
            r += poids[i] * (1.0 - distances[i])
        return r
    
    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        """
        Entrée:
            sensors[i]      : distance normalisée (0 proche, 1 loin)
            sensor_view[i]  : 0 vide, 1 mur, 2 robot
            sensor_team[i]  : équipe si robot détecté sinon "n/a"
        Sortie:
            translation     : vitesse translation dans [-1, 1]
            rotation        : vitesse rotation dans [-1, 1]
            ask_for_reset   : False (on ne reset pas ici)
        """

        sensor_to_wall = []
        sensor_to_robot = []
        for i in range (0,8):
            if  sensor_view[i] == 1:
                sensor_to_wall.append( sensors[i] )
                sensor_to_robot.append(1.0)
            elif  sensor_view[i] == 2:
                sensor_to_wall.append( 1.0 )
                sensor_to_robot.append( sensors[i] )
            else:
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(1.0)
        
        dF = sensors[sensor_front]
        dFL = sensors[sensor_front_left]
        dFR = sensors[sensor_front_right]
        dL = sensors[sensor_left]
        dR = sensors[sensor_right]

        # Rôles
        rid = self.robot_id % 4
        est_robot0_GA = (rid == 0)      # Robot 0: coverage avec poids "GA"
        est_robot1_mur = (rid == 1)     # Robot 1: suit murs/couloirs (coverage stable)
        est_robot2_pixels = (rid == 2)  # Robot 2: "max pixels" (avance vite si possible)
        est_robot3_agresif = (rid == 3) # Robot 3: suit les autres robots agressivement

        #Evitement murs
        POIDS_MUR_STD = [0.10, 0.90, 0.60, 0.05, 0.00, 0.05, -0.60, -0.90]

        #Répulsion allié (anti-train)
        POIDS_ALLIE_REP = [0.00, -0.70, -0.50, 0.00, 0.00, 0.00, 0.50, 0.70]

        #Poids "GA" (Robot 0) optimisés 
        POIDS_MUR_GA = [1.0, -1.0, 0.0, 1.0, 1.0, 1.0, -1.0, -1.0]


        def ennemi_dans_arc(indices):
            """True si un ennemi est vu par ces capteurs."""
            if sensor_view is None or sensor_team is None:
                return False
            for i in indices:
                if sensor_view[i] == 2 and sensor_team[i] != "n/a" and sensor_team[i] != self.team_name:
                    return True
            return False

        def allie_trop_proche_devant():
            """évite que nos robots se collent."""
            if sensor_view is None or sensor_team is None:
                return False
            for i in [sensor_front, sensor_front_left, sensor_front_right]:
                if sensor_view[i] == 2 and sensor_team[i] == self.team_name and sensors[i] < 0.5:
                    return True
            return False

        def biais_ennemi_gauche_droite():
            """
            Score direction:
              >0 => ennemi à gauche
              <0 => ennemi à droite
            """
            if sensor_view is None or sensor_team is None:
                return 0.0
            score = 0.0
            for i in [sensor_left, sensor_front_left]:
                if sensor_view[i] == 2 and sensor_team[i] != "n/a" and sensor_team[i] != self.team_name:
                    score += (1.0 - sensors[i])
            for i in [sensor_right, sensor_front_right]:
                if sensor_view[i] == 2 and sensor_team[i] != "n/a" and sensor_team[i] != self.team_name:
                    score -= (1.0 - sensors[i])
            return score


        # Lecture mémoire
        mode, timer, last_dist = self.lire_memoire(self.memory)

        current_dist = self.log_sum_of_translation
        deplacement = abs(current_dist - last_dist)
        print(f"Robot {self.robot_id} | Mod: {mode} | Dep: {deplacement:.3f} | dF: {dF:.2f}")

        if mode != 3 and deplacement < 0.1:
            print(f"Robot {self.robot_id} Sıkıştı")
            timer += 2
            if timer > 40:
                mode = 3
                timer = 45
        elif mode == 0:
            timer = 0

        # Gestion des autres modes
        if mode != 0:
            if timer > 0: timer -= 1
            else: mode = 0
        
        # PRIORITE 0 : EVITEMENT DE SE COINCER
        if mode == 3:
            # Verifie a quel point la voie devant est libre
            libre = sensor_to_wall[sensor_front] > 0.6
            # reculer 10 pas
            if timer > 35:
                translation = -0.3
                rotation = 0.2
            # tourner jusqu'a la voie soit libre
            elif not libre and timer > 1:
                translation = 0.0
                rotation = 1.0
            # sortir de la manoeuvre si la voie est libre ou si le temps est écoulé
            else:
                translation = 0.6
                rotation = 0.0
                if timer <= 1: 
                    mode = 0

            self.memory = self.ecrire_memoire(mode, timer, current_dist)
            return translation, rotation, False

        
        # PRIORITE 1 : EVITEMENT DE MUR
        mur_proche = (sensor_to_wall[sensor_front] < 0.30) or (sensor_to_wall[sensor_front_left] < 0.25) or (sensor_to_wall[sensor_front_right] < 0.25)
        mur_urgence = (sensor_to_wall[sensor_front] < 0.18) or (sensor_to_wall[sensor_front_left] < 0.15) or (sensor_to_wall[sensor_front_right] < 0.15)
    
        if mur_urgence and mode == 0 and not est_robot1_mur:
            mode = 1
            timer = 15

        if mode == 1 and timer > 0:
            left = (1.0 - sensor_to_wall[sensor_front_left]) * 1.4 + (1.0 - sensor_to_wall[sensor_left]) * 0.9
            right = (1.0 - sensor_to_wall[sensor_front_right]) * 1.4 + (1.0 - sensor_to_wall[sensor_right]) * 0.9
            
            translation = 0.3
            rotation = 1.2 * (right - left) + 0.02
            
            self.memory = self.ecrire_memoire(mode, timer, current_dist)
            return translation, rotation, False

        # PRIORITE 2 : CHASSE D'ENNEMI
        ennemi_vu = ennemi_dans_arc([sensor_front, sensor_front_left, sensor_front_right, sensor_left, sensor_right])
        if ennemi_vu:
            mode = 2
            dir_ennemi = biais_ennemi_gauche_droite()
            rot_mur = self.braitenberg_rotation(sensors, POIDS_MUR_STD)

            w_hunt, w_wall = 1.0, 0.3
            vitesse = 0.60

            if est_robot3_agresif:
                w_hunt, w_wall = 1.2, 0.2
                vitesse = 0.60
            elif est_robot0_GA:
                w_hunt, w_wall = 0.8, 0.5
                vitesse = 0.60
            elif est_robot1_mur:
                w_hunt, w_wall = 0.5, 0.8
                vitesse = 0.65

            rotation = self.borne(w_wall * rot_mur + w_hunt * dir_ennemi)

            if sensor_to_robot[sensor_front] < 0.25: 
                vitesse *= 0.4

            if allie_trop_proche_devant():
                evitement = -0.7 if dir_ennemi > 0 else 0.7
                rotation = self.borne(rotation + evitement)
                vitesse *= 0.8

            self.memory = self.ecrire_memoire(mode, timer, current_dist)
            return vitesse, rotation, False
        
        # PRIORITE 3 : COMPORTEMENT NORMAL
        else:
            if est_robot1_mur:
                if (timer + self.robot_id) % 200 < 8:
                    translation = 0.7
                    rotation = 0.0
                    self.memory = self.ecrire_memoire(mode, timer, current_dist)
                    return translation, rotation, False
                
                target_dist = 0.35
                
                if sensor_to_wall[sensor_right] < 0.8:
                    error = target_dist - sensor_to_wall[sensor_right]
                    rotation = error * 5.5
                    translation = 0.65
                    
                    if sensor_to_wall[sensor_front] < 0.2 or sensor_to_wall[sensor_front_left] < 0.2:
                        rotation = 0.8
                        translation = 0.4
                
                else:
                    translation = 0.6
                    bias = -0.1 if timer % 40 < 30 else 0.1 
                    rotation = self.borne(0.9 * (sensor_to_wall[sensor_front_left] - sensor_to_wall[sensor_front_right])) + bias
                
                self.memory = self.ecrire_memoire(mode, timer, current_dist)
                return self.borne(translation), self.borne(rotation), False
            else:
                if est_robot2_pixels:
                    base_speed = 0.78
                elif est_robot0_GA:
                    base_speed = 0.66 
                elif est_robot3_agresif:
                    base_speed = 0.70
                else:
                    base_speed = 0.64
        
                clear = min(dF, dFL, dFR)
                translation = base_speed * (0.3 + 0.7 * clear)
        
                if est_robot3_agresif:
                    rotation = self.borne(1.1 * (dFL - dFR))
                else:
                    rotation = self.borne(0.9 * (dFL - dFR))
        
                if est_robot0_GA:
                    rotation = self.borne(rotation + 0.55 * self.braitenberg_rotation(sensors, POIDS_MUR_GA))
                else:
                    rotation = self.borne(rotation + 0.55 * self.braitenberg_rotation(sensors, POIDS_MUR_STD))
        
                if est_robot2_pixels:
                    if dF > 0.75 and dFL > 0.65 and dFR > 0.65:
                        translation = 0.90
                    rotation = self.borne(rotation + 0.08)
        
                if mur_proche:
                    translation *= 0.75
        
                if allie_trop_proche_devant():
                    translation *= 0.75
                    rotation = self.borne(rotation + 1.0 * self.braitenberg_rotation(sensors, POIDS_ALLIE_REP))
        
                self.memory = self.ecrire_memoire(mode, timer, current_dist)
                return self.borne(translation), self.borne(rotation), False
