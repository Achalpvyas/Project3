#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math

# # publishers
pub = None
vect = Twist()


def calcMotion(x0,y0,x1,y1):
    ydiff = y1 - y0
    xdiff = x1 - x0
    ratio = (ydiff)/(xdiff)

    angle = math.atan2(ydiff,xdiff)
    print(angle)
    distance = math.sqrt(ydiff**2 + xdiff**2)
    # print(angle, distance)
    return angle, distance


if __name__ == '__main__':

    rospy.init_node('planner')

    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    
    rate = rospy.Rate(5)

    f = open("/src/pathplan/scripts/pathfile1.txt", "r+")
    lines = f.readlines()
    # print(lines)
    values = [line.rstrip().split(' ') for line in lines]
    xcoord = [float(i[0]) for i in values]
    ycoord = [float(i[1]) for i in values]
    theta = [float(i[1]) for i in values]

    for i in range(len(xcoord)):

        angles, displacement = calcMotion(xcoord[i-1], ycoord[i-1], xcoord[i], ycoord[i])

        vect.linear.x = displacement
        vect.angular.z = angles

        pub.publish(vect)
        rate.sleep()
 
   


  








# if __name__ == '__main__':
#     main()
