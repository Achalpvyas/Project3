"""
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
  
  goal_point = eval(input('Please enter the goal coordinates in this format - [X_coord, Y_coord]:'))
  while not a_star_algo.check_node(goal_point, radius+clearance):
    goal_point = eval(input('Please enter the goal coordinates in this format - [X_coord, Y_coord]:'))
  print('The goal point you entered is:', goal_point)
  print('')
  goal_circle = plt.Circle((goal_point[0], goal_point[1]), radius= 1.5,fill=False)
  plt.gca().add_patch(goal_circle)
  
  
   # Taking the 2 wheel RPM values from the user
  RPM_1 = eval(input('Please enter the first RPM value :'))
  print('The start point you entered is:', RPM_1)
  print('')

  RPM_2 = eval(input('Please enter the  RPM value :'))
  print('The start point you entered is:', RPM_2)
  print('')

  def plot_curve(X0,Y0,Theta0,UL,UR):  
  t=0 
  r=0.1 
  L=1 
  dt=0.1
  X1=0
  Y1=0 
  dtheta=0
  Theta0=3.14*Theta0/180
  Theta1=Theta0 
  while t<1:
  t=t+dt 
  X0=X0+X1
  Y0=Y0+Y1 
  dx=r*(UL+UR)*math.cos(Theta1)*dt
  dy=r*(UL+UR)*math.sin(Theta1)*dt       
  dtheta=(r/L)*(UR-UL)*dt
  X1=X1+dx   
  Y1=Y1+dy
  Theta1=Theta1+0.5*dtheta
  plt.quiver(X0, Y0, X1, Y1,units='xy' ,scale=1,color= 'r',width =0.2, headwidth = 1,headlength=0)
  Xn=X0+X1
  Yn=Y0+Y1 
  Thetan=180*(Theta1)/3.14 
  return Xn,Yn,Thetan
