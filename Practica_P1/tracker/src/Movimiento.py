#!/usr/bin/env python
# -- coding: utf-8 --

import rospy, math, time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf.transformations
from geometry_msgs.msg import Point
import numpy as np

timeAnt = 0.0
x = y = 0


def getCoors(msg):
    global x,y
    x = msg.x
    y = msg.y


rospy.init_node('prueba', anonymous=True)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
sub = rospy.Subscriber('/Targeted_Person',Point,getCoors)

twist = Twist()

while not rospy.is_shutdown():
    Kl, Kw = 0.2, 0.2

    print 'x:',x
    print 'y:',y

    error = (math.atan2(y, x)) % (2*math.pi) - math.pi # Angulo normalizado: [-pi,pi]
    dist = math.hypot(x, y)

    # Control proporcional
    if  dist > 0.5:
        twist.linear.x = dist * Kl if abs(error) < 0.2 else dist * Kl * 0.25
    else: 
        twist.linear.x = 0.0
        
    error = error if error != 0.0 else 1e-9
    twist.angular.z = (error/abs(error)) * 0.4

    rospy.sleep(0.1)

    print 'distancia', dist
    print 'error', error
    print 'lineal',twist.linear.x 
    print 'angular',twist.angular.z

    # Publicar siempre (si hubo excepción, twist es cero)
    pub.publish(twist)