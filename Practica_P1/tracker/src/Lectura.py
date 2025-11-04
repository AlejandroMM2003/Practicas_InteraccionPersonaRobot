#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
import tf, numpy as np

# LANZAR PRIMERO ROSBAG (JUNTO AL ROSCORE EN OTRA TERMINAL) Y DESPUES LAUNCH

# Variables globales del programa
PersonaSeguida = ""     
persona_Ant = {}        # Personas en t-1
Filtrados = {}          # Personas en t
MaxSpeed = 5            # km/h
MaxPersons = 5          # Maximo de personas en escena
alfa = 0.0              # Coeficiente de ponderacion


Send_marks = rospy.Publisher('/My_Marks', MarkerArray, queue_size=10)
Send_Follow = rospy.Publisher('/Targeted_Person', Point, queue_size=10)


def EnviarMarcas(People_Dict,seguido):
    global Send_marks
    ArrayMarks = MarkerArray()

    for nome,person in People_Dict.items():
        mark = Marker()
        mark.header.frame_id = "base_link"
        mark.id = hash(nome) % 1000
        mark.header.stamp = person.header.stamp
        mark.type = Marker.TEXT_VIEW_FACING
        mark.action = Marker.ADD
        mark.text = nome
        dir = math.atan2(person.pos.y , person.pos.x)

        ox,oy,oz,ow = quaternion_from_euler(0,0,dir)

        mark.pose.position.x, mark.pose.position.y, mark.pose.position.z = person.pos.x , person.pos.y , 1.0
        mark.pose.orientation.x, mark.pose.orientation.y, mark.pose.orientation.z, mark.pose.orientation.w = \
            ox , oy , oz , ow

        mark.scale.x, mark.scale.y, mark.scale.z = 0.25 , 0.25 , 0.25

        if nome == seguido:
            mark.color.r, mark.color.g, mark.color.b, mark.color.a = 1.0 , 1.0 , 0.0 , 1.0
        else:
            mark.color.r, mark.color.g, mark.color.b, mark.color.a = 1.0 , 0.0 , 0.0 , 1.0

        mark.lifetime = rospy.Duration(0.5)
        ArrayMarks.markers.append(mark)


    Send_marks.publish(ArrayMarks)





def callback_filtrado(data,MaxTargets):
    # Variables globales
    global PersonaSeguida, persona_Ant, Filtrados, Ponderados, alfa

    if not PersonaSeguida or PersonaSeguida not in Filtrados.keys():
        persona = data.people
        Distancias = [math.hypot(p.pos.x , p.pos.y) for p in persona]
        idx = Distancias.index(min(Distancias))
        PersonaSeguida = persona[idx].name
        persona = {person.name : person for person in persona}

    Max_vel = (MaxTargets[0]*1000.0)/3600.0 # m/s --> Maxima velocidad permitida

    persona = {person.name : person for person in data.people}


    # Iteramos todas con todas
    for new_id,new_person in persona.items():
        # Solo añadimos o actualizamos las primeras personas en aparecer
        if len(Filtrados) < MaxTargets[1] and new_id not in Filtrados.keys():
            Filtrados[new_id] = new_person
        for old_id,old_person in persona_Ant.items():
            # Descubrimos las coordenadas de las anteriores respecto a las nuevas
            Xant = old_person.pos.x
            Xnew = new_person.pos.x
            Yant = old_person.pos.y
            Ynew = new_person.pos.y

            # Calculamos la distancia entre ellas
            Delta_d = math.hypot((Xnew-Xant) , (Ynew-Yant))

            # Calculamos el tiempo entre estas y con ello la distancia maxima permitida
            Delta_t = abs(new_person.header.stamp.to_sec() - old_person.header.stamp.to_sec())
            PersonSpeed = Delta_d / Delta_t if Delta_t != 0.0 else Delta_d / 1e-9 # v = d/t, con t > 0

            # Filtrado por cercania a lecturas anteriores
            if 1e-6 < PersonSpeed < Max_vel:
                Filtrados[old_id] = new_person

    # Buffer de personas en t-1
    persona_Ant = Filtrados.copy()
    # datos para el global
    alfa = 0.0
    Ponderados = Filtrados
    EnviarMarcas(Ponderados, PersonaSeguida)


    
def PredecirEstados(PersonT,PersonTm1):
    PredictFilt = {}
    # Iteramos todas con todas
    for idT in set(PersonT.keys()) & set(PersonTm1.keys()):
        # Para la misma persona
        XPred = 2*PersonT[idT].pos.x - PersonTm1[idT].pos.x # Calculo de derivada discreta en x
        YPred = 2*PersonT[idT].pos.y - PersonTm1[idT].pos.y # Calculo de derivada discreta en y
        PredPerson = PersonT[idT]
        PredPerson.pos.x = XPred
        PredPerson.pos.y = YPred
        PredictFilt[idT] = PredPerson

    return PredictFilt


def Ponderar(Pred,Med,alfa):
    # Iteramos todos con todos
    Ponderados = {}
    for pred_id,pred_person in Pred.items():
        for med_id,med_person in Med.items():
            # Para la misma persona
            if pred_id == med_id:
                PersonaPonderada = pred_person
                PersonaPonderada.pos.x = alfa * pred_person.pos.x + (1-alfa) * med_person.pos.x
                PersonaPonderada.pos.y = alfa * pred_person.pos.y + (1-alfa) * med_person.pos.y
                Ponderados[pred_id] = PersonaPonderada
    
    return Ponderados



PredAnt = {}

def timer_callback(event):
    global alfa, Ponderados, Send_Follow, PredAnt
    if Filtrados and persona_Ant and PersonaSeguida:
        # Usamos Filtrado como etapa anterior si no hay muchas iteraciones sin medidas
        Predict = PredecirEstados(Filtrados if alfa < 0.02 else PredAnt, persona_Ant)
        alfa = min(alfa + 0.01, 1.0)
        # Lo mismo que antes
        Ponderados = Ponderar(Predict, Filtrados if alfa < 0.02 else PredAnt, alfa)
        PredAnt = Ponderados
        EnviarMarcas(Ponderados, PersonaSeguida)

        # Enviar a movimiento (coordenas relativas al robot)
        xFollow, yFollow = Ponderados[PersonaSeguida].pos.x, Ponderados[PersonaSeguida].pos.y

        Punto = Point()
        Punto.x = xFollow
        Punto.y = yFollow
        Send_Follow.publish(Punto)

def main():
    global Send_Follow
    rospy.init_node('PruebaDatos', anonymous=True)
    rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray,
                     callback_filtrado, callback_args=(MaxSpeed,MaxPersons))
    
    # Timer ejecuta la predicción y el envío de marcas cada 0.1s, si no usaramos timer solo ejecutaria en cada medicion
    Timer = rospy.Timer(rospy.Duration(0.5), timer_callback)
    rospy.sleep(0.1)
    Timer.run()






    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass