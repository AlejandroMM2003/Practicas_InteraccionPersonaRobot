#!/usr/bin/env python
# -- coding: utf-8 --

'''
Nodo ROS que controla el movimiento de un robot móvil (p. ej. Turtlebot)
para seguir a una persona detectada. Recibe las coordenadas relativas
de la persona respecto al robot mediante el topic /Targeted_Person
y genera comandos de velocidad lineal y angular para acercarse a ella.
'''

import rospy, math, time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf.transformations
from geometry_msgs.msg import Point
import numpy as np


# ============================================================
# === INCIALIZACION DE VARIABLES =============================
# ============================================================

x = y = 0

# ============================================================
# === CALLBACK: getCoors =====================================
# ============================================================

def getCoors(msg):
    """
    Callback ejecutado cada vez que se recibe un mensaje en /Targeted_Person.
    Actualiza las coordenadas x, y de la persona respecto al robot.
    """

    global x,y
    x = msg.x
    y = msg.y

# ============================================================
# === CONFIGURACIÓN DE NODO ROS ===============================
# ============================================================

rospy.init_node('prueba', anonymous=True)                                       # Inicializa el nodo con nombre 'prueba'
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)   # Publicador: velocidades de movimiento del robot
sub = rospy.Subscriber('/Targeted_Person',Point,getCoors)                       # Suscriptor: recibe la posición relativa de la persona
twist = Twist()                                                                 # Objeto Twist que almacenará las velocidades que se publicarán

# ============================================================
# === BUCLE PRINCIPAL ========================================
# ============================================================

while not rospy.is_shutdown():
    Kl, Kw = 0.2, 0.2

    print 'x:',x
    print 'y:',y

    # --------------------------------------------------------
    # Cálculo del ángulo de error entre el eje del robot (x)
    # y la posición de la persona (x, y)
    # --------------------------------------------------------

    error = (math.atan2(y, x)) % (2*math.pi) - math.pi # Angulo normalizado: [-pi,pi]
    dist = math.hypot(x, y)

    # --------------------------------------------------------
    # CONTROL PROPORCIONAL
    # --------------------------------------------------------

    if  dist > 0.5:
        twist.linear.x = dist * Kl if abs(error) < 0.2 else dist * Kl * 0.25
    else: 
        twist.linear.x = 0.0
        
    error = error if error != 0.0 else 1e-9 # Ternario para evitar division entre 0
    twist.angular.z = (error/abs(error)) * 0.4

    rospy.sleep(0.1)

    # --------------------------------------------------------
    # Depuracion
    # --------------------------------------------------------

    print 'distancia', dist
    print 'error', error
    print 'lineal',twist.linear.x 
    print 'angular',twist.angular.z

    # --------------------------------------------------------
    # Publicar velocidades en el topic de control del robot
    # --------------------------------------------------------

    pub.publish(twist)