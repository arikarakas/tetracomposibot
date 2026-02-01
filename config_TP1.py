# Configuration file.

import arenas

# general -- first three parameters can be overwritten with command-line arguments (cf. "python tetracomposibot.py --help")

display_mode = 0
arena = 1
position = False 
max_iterations = 1001 #401*500

# affichage

display_welcome_message = False
verbose_minimal_progress = True # display iterations
display_robot_stats = False
display_team_stats = False
display_tournament_results = False
display_time_stats = True

# initialization : create and place robots at initial positions (returns a list containing the robots)

import robot_dumb
import robot_braitenberg_avoider
import robot_braitenberg_loveWall
import robot_braitenberg_hateWall
import robot_braitenberg_loveBot
import robot_braitenberg_hateBot
import robot_subsomption

def initialize_robots(arena_size=-1, particle_box=-1): # particle_box: size of the robot enclosed in a square
    x_center = arena_size // 2 - particle_box / 2
    y_center = arena_size // 2 - particle_box / 2
    robots = []
    #robots.append(robot_braitenberg_avoider.Robot_player(55, y_center, 0, name="Avoider", team="A"))
    #robots.append(robot_braitenberg_loveWall.Robot_player(40, 55, 30, name="Love Wall", team="B"))
    #robots.append(robot_braitenberg_hateWall.Robot_player(x_center, 90, 90, name="Hate Wall", team="C"))
    #robots.append(robot_braitenberg_loveBot.Robot_player(4, y_center, 0, name="Love Bot", team="D"))
    #robots.append(robot_braitenberg_hateBot.Robot_player(90, 49, 180, name="Hate Bot", team="E"))
    robots.append(robot_subsomption.Robot_player(40, 55, 30, name="Sub1", team="B"))
    robots.append(robot_subsomption.Robot_player(x_center, 90, 90, name="Sub2", team="C"))
    robots.append(robot_subsomption.Robot_player(4, y_center, 0, name="Sub3", team="D"))
    robots.append(robot_subsomption.Robot_player(90, 49, 180, name="Sub4", team="E"))
    return robots
