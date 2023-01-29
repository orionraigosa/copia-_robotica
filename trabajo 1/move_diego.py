#!/usr/bin/env python3
from cmath import inf
import sys
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self):
        self.ctrl_c = False
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador

    # inicia funciòn publicacion
    def velPublish(self, cmd):
        while not self.ctrl_c:
            connections = self.velPublisher.get_num_connections()
            if connections > 0:
                self.velPublisher.publish(cmd)
                break
            else:
                self.rate.sleep()

    # funciòn para mover y girar robot
    def move(self, movingTime, linear_velocity, angular_velocity): # Se crea una funcion de movimiento
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.velPublish(cmd)
        time.sleep(movingTime)

class scanner:
    def __init__(self):
        self.robot = MoveRobot()
        self.initScan() # Define una propiedad para inicial el escaneo
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor

    # funcion de scaneo
    def initScan(self):
        self._scan = None
        while self._scan is None:
            try:
                #se conecta al nodo de comunicacion para escuchar mensajes
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1)
                return True
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying")

        rospy.loginfo("scan topic successfully")

    def execute(self):
        print("")


    def scanCallback(self,msg):
        self.distances = msg.ranges

        #variable de entrada distancia medida por el sensor
        distancia_medida = ctrl.Antecedent(np.arange(0.5, 2.5, 0.5), 'distancia') #Entrada

        #variables de salida, definicion de rangos en los cuales pueden operar
        #se tiene la velocidad lineal(velocidad) y la velocidad angular(girar)
        velocidad = ctrl.Consequent(np.arange(-2.0, 2.0, 0.5), 'velocidad')
        girar = ctrl.Consequent(np.arange(0,1.5,0.1), 'girar')
        # girar = ctrl.Consequent(np.arange(-0.8, 0.8,0.1),'girar') # Salida, entrega la velocidad angular

        #definicion de rangos de calificacioin para la mediciòn de la distancia
        distancia_medida['Muycerca'] = fuzz.trimf(distancia_medida.universe,[0.2,0.5,0.7])
        distancia_medida['cerca'] = fuzz.trimf(distancia_medida.universe,[0.5,0.7,3.0])
        distancia_medida['lejos'] = fuzz.trimf(distancia_medida.universe,[0.7,3.0,3.0])

        #definicion de rangos de calificacioin para calcular la velocidad
        velocidad['Freno'] = fuzz.trimf(velocidad.universe,[-1.0,-1.0,0.0])
        velocidad['Medio'] = fuzz.trimf(velocidad.universe,[-1.0,1.8,2.0])
        velocidad['Rapido'] = fuzz.trimf(velocidad.universe,[1.0,2.0,4.0])

        #definicion de rangos de calificacioin para girar
        girar['Muypoco'] = fuzz.trimf(girar.universe,[0,0,0.8])
        girar['Poco'] = fuzz.trimf(girar.universe,[0,0.8,1.5])
        girar['Mucho'] = fuzz.trimf(girar.universe,[0.8,1.5,1.5])

        # girar['Muypoco'] = fuzz.trimf(girar.universe,[-0.8, -0.8, 0.0]) # En ese rango de datos debe girar en los valores de Muypoco
        # girar['Poco'] = fuzz.trimf(girar.universe,[-0.8, 0.0, 0.8]) # En ese rango de datos debe girar Poco
        # girar['Mucho'] = fuzz.trimf(girar.universe,[0.0, 0.8, 0.8])

        # definiciòn de reglas para calcular la velocidad lineal y angular
        # de acuerdo a la distacia medida por el sensor

        #reglas para velocidad lineal
        rule1 = ctrl.Rule(distancia_medida['Muycerca'],velocidad['Freno'])
        rule2 = ctrl.Rule(distancia_medida['cerca'],velocidad['Medio'])
        rule3 = ctrl.Rule(distancia_medida['lejos'], velocidad['Rapido'])

        #regalas para velocidad angular o giro
        rule4 = ctrl.Rule(distancia_medida['lejos'], girar['Muypoco'])
        rule5 = ctrl.Rule(distancia_medida['cerca'], girar['Poco'])
        rule6 = ctrl.Rule(distancia_medida['Muycerca'],girar['Mucho'])

        #se define controlador para calcular la velocidad lineal
        velocidad_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])

        # Simulamos el sistema de control
        simulador_velocidad = ctrl.ControlSystemSimulation(velocidad_ctrl)

        for i in range(-15,15):
            simulador_velocidad.input['distancia'] = self.distances[i]
            # Se procesan los datos y se obtiene el resultado
            simulador_velocidad.compute() # Se operan los valores obtenidos
            valor_velocidad = simulador_velocidad.output['velocidad'] # Se le asigna al mensaje la salida
            print(simulador_velocidad.output['velocidad']) # Se imprime el valor de salida

        # Creamos un sistema de control
        # tipp_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])

        if i in range(-30,-90):
            simulador_velocidad.input['distancia'] = self.distances[i]
            # Se procesan los datos y se obtiene el resultado
            simulador_velocidad.compute() # Se operan los valores obtenidos
            valor_velocidad = simulador_velocidad.output['velocidad'] # Se le asigna al mensaje la salida
            print(simulador_velocidad.output['velocidad']) # Se imprime el valor de salida

        # Creamos un sistema de control
        # tipp_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])

        # tipp = ctrl.ControlSystemSimulation(tipp_ctrl)

        if i in range(30,90):
            simulador_velocidad.input['distancia'] = self.distances[i]
            # Se procesan los datos y se obtiene el resultado
            simulador_velocidad.compute() # Se operan los valores obtenidos
            valor_velocidad = simulador_velocidad.output['velocidad'] # Se le asigna al mensaje la salida
            print(simulador_velocidad.output['velocidad']) # Se imprime el valor de salida

        # Creamos un sistema de control
        # tipp_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])

        # tipp = ctrl.ControlSystemSimulation(tipp_ctrl)

        if i in range(-180,180):
            simulador_velocidad.input['distancia'] = self.distances[i]
            # Se procesan los datos y se obtiene el resultado
            simulador_velocidad.compute() # Se operan los valores obtenidos
            valor_velocidad = simulador_velocidad.output['velocidad'] # Se le asigna al mensaje la salida
            print(simulador_velocidad.output['velocidad']) # Se imprime el valor de salida

        # Creamos un sistema de control
        girar_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])
        simulador_girar = ctrl.ControlSystemSimulation(girar_ctrl)

        if i in range(-15,15):
            simulador_girar.input['distancia'] = self.distances[i]
            simulador_girar.compute() # Se operan los valores obtenidos
            valor_giro = simulador_girar.output['girar'] # Se le asigna al mensaje la salida

        if i in range(-45,-90):
            simulador_girar.input['distancia'] = self.distances[i]
            simulador_girar.compute() # Se operan los valores obtenidos
            valor_giro = simulador_girar.output['girar'] # Se le asigna al mensaje la salida

        if i in range(45,90):
            simulador_girar.input['distancia'] = self.distances[i]
            simulador_girar.compute() # Se operan los valores obtenidos
            valor_giro = simulador_girar.output['girar'] # Se le asigna al mensaje la salida

        if i in range(-180,180):
            simulador_girar.input['distancia'] = self.distances[i]
            simulador_girar.compute() # Se operan los valores obtenidos
            valor_giro = simulador_girar.output['girar'] # Se le asigna al mensaje la salida

        print("angular: " + str(valor_giro))

        self.robot.move(0.01, valor_velocidad, valor_giro)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()