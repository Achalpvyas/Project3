Path Planning for Rigid Robot using A* Algorithm
Authors:
Achal Vyas (avyas@umd.edu)
Pruthvi 
Graduate Students pursuing Masters in Robotics,
University of Maryland, College Park
"""
import matplotlib.pyplot as plt
import numpy as np
import custom_map
import a_star_algo
import node

# Main Function
def main():
  # Taking the obstacle clearance and the robot radius from the user
  clearance = eval(input('Enter the clearance of the robot from the obstacle:'))
  print('The clearance value you entered is:', clearance)
  print('')
  
  radius = eval(input('Enter the robot radius:'))
  print('The radius value you entered is:', radius)
  print('')
  
  # Taking the Start Point and Goal Points from the user
  start_point = eval(input('Please enter the start coordinates in this format - [X_coord, Y_coord, Theta]:'))
  while not a_star_algo.check_node(start_point, radius+clearance):
    start_point = eval(input('Please enter the start coordinates in this format - [X_coord, Y_coord, Theta]:'))
  print('The start point you entered is:', start_point)
  print('')
  start_circle = plt.Circle((start_point[0], start_point[1]), radius= radius+clearance, fc='g')
  plt.gca().add_patch(start_circle)
  
  goal_point = eval(input('Please enter the goal coordinates in this format - [X_coord, Y_coord, Theta]:'))
  while not a_star_algo.check_node(goal_point, radius+clearance):
    goal_point = eval(input('Please enter the goal coordinates in this format - [X_coord, Y_coord, Theta]:'))
  print('The goal point you entered is:', goal_point)
  print('')
  goal_circle = plt.Circle((goal_point[0], goal_point[1]), radius= 1.5,fill=False)
  plt.gca().add_patch(goal_circle)
