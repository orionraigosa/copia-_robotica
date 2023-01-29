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

class moveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self):
        self.ctrl_c = False 
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador

    def publishOnceCmdVel(self, cmd): # Se crea la funcion de publicacion
        while not self.ctrl_c:
            connections = self.velPublisher.get_num_connections()
            if connections > 0:
                self.velPublisher.publish(cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def move(self, movingTime, linear1, angular1): # Se crea una funcion de movimiento
        cmd = Twist()
        cmd.linear.x = linear1
        cmd.angular.z = angular1
        
        self.publishOnceCmdVel(cmd)
        time.sleep(movingTime)

class scanner:
    def __init__(self):
        self.moveObject = moveRobot()
        self.initScan() # Define una propiedad para inicial el escaneo
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback) # Creo el subsciptor

    def initScan(self): # Inicio el escaneo
        self._scan = None # Creo una variable de control
        while self._scan is None: # Mientras la condicion se cumpla
            try:                
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) # Se espera a la lectura de un mensaje
                return True
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg):
        self.distances = msg.ranges
        dist = self.distances[0]
        print(dist)

        distancia = ctrl.Antecedent(np.arange(0.5,2.5,0.5), 'distancia') #Entrada
        velocidad = ctrl.Consequent(np.arange(-1.0,1.5,0.5), 'velocidad') #Salida de la velocidad lineal
        gira = ctrl.Consequent(np.arange(-1.5,1.5,0.3),'gira') # Salida, entrega la velocidad angular

        distancia['Muycerca'] = fuzz.trimf(distancia.universe,[0.2,0.5,0.7]) # En ese rango de datos se encuentra muy cerca
        distancia['cerca'] = fuzz.trimf(distancia.universe,[0.5,0.7,3.0]) # En ese rango de datos se encuentra cerca
        distancia['lejos'] = fuzz.trimf(distancia.universe,[0.7,3.0,3.0]) # En ese rango de datos se encuentra lejos
        # Defino variables linguisticas para la salida de velocidad lineal
        velocidad['reversa'] = fuzz.trimf(velocidad.universe,[-1.0,-1.0,0.0]) # En ese rango de datos debe frenar
        velocidad['Medio'] = fuzz.trimf(velocidad.universe,[-1.0,1.2,1.5]) # En ese rango de datos debe moverse poco
        velocidad['Rapido'] = fuzz.trimf(velocidad.universe,[1.2,1.5,1.5]) # En ese rango de datos debe ir rapido
	# Defino variables linguisticas para la salida de velocidad angular
        gira['right'] = fuzz.trimf(gira.universe,[-1.5,-1,-0.5]) # En ese rango de datos debe girar en los valores de Muypoco
        gira['lento'] = fuzz.trimf(gira.universe,[-1,-0.5,0.5]) # En ese rango de datos debe girar Poco
        gira['left'] = fuzz.trimf(gira.universe,[0.5,1,1.5]) # En ese rango de datos debe girar Mucho

        # Creación de las reglas de inferencia o decisiones del robot autónomo
        rule1 = ctrl.Rule(distancia['Muycerca'],velocidad['reversa'])
        rule2 = ctrl.Rule(distancia['cerca'],velocidad['Medio'])
        rule3 = ctrl.Rule(distancia['lejos'], velocidad['Rapido'])
        rule4 = ctrl.Rule(distancia['lejos'], gira['lento'])
        rule5 = ctrl.Rule(distancia['Muycerca'], gira['right'])
        rule6 = ctrl.Rule(distancia['Muycerca'], gira['left'])
    
        

        vel_lineal = ctrl.ControlSystem([rule1, rule2, rule3])

        # Simulamos el sistema de control
        vel = ctrl.ControlSystemSimulation(vel_lineal)

        for i in range(-10,10):
                vel.input['distancia'] = self.distances[i]
                # Se procesan los datos y se obtiene el resultado
                vel.compute() # Se operan los valores obtenidos
                linear2=vel.output['velocidad'] # Se le asigna al mensaje la salida
                print(vel.output['velocidad']) # Se imprime el valor de salida
                

        if i in range(0,-90):
                vel.input['distancia'] = self.distances[i]
                # Se procesan los datos y se obtiene el resultado
                vel.compute() # Se operan los valores obtenidos
                linear2=vel.output['velocidad'] # Se le asigna al mensaje la salida
                print(vel.output['velocidad']) # Se imprime el valor de salida
                

        
        if i in range(0,90):
                vel.input['distancia'] = self.distances[i]
                # Se procesan los datos y se obtiene el resultado
                vel.compute() # Se operan los valores obtenidos
                linear2=vel.output['velocidad'] # Se le asigna al mensaje la salida
                print(vel.output['velocidad']) # Se imprime el valor de salida
                
        if i in range(-180,180):
        	vel.input['distancia'] = self.distances[i]
        	# Se procesan los datos y se obtiene el resultado
        	vel.compute() # Se operan los valores obtenidos
        	linear2=vel.output['velocidad'] # Se le asigna al mensaje la salida
        	print(vel.output['velocidad']) # Se imprime el valor de salida
                
      
        # Creamos un sistema de control 
        ang_right_ctrl = ctrl.ControlSystem([rule4, rule5])
        ang_right = ctrl.ControlSystemSimulation(ang_right_ctrl)
        
        ang_left_ctrl = ctrl.ControlSystem([rule4, rule6])
        ang_left = ctrl.ControlSystemSimulation(ang_left_ctrl)
    
        
        for i in range(-60,-30):
        	ang_right.input['distancia'] = self.distances[i]
        	ang_right.compute() # Se operan los valores obtenidos
        	angular2 =ang_right.output['gira'] # Se le asigna al mensaje la salida
    
        
        if i in range(30,60):     	                
          
                ang_left.input['distancia'] = self.distances[i]
                ang_left.compute() # Se operan los valores obtenidos
                angular2=ang_left.output['gira'] # Se le asigna al mensaje la salida
              
                
        if i in range(0,-180):        
        	ang_right.input['distancia'] = self.distances[i]
        	ang_right.compute() # Se operan los valores obtenidos
        	angular2 =ang_right.output['gira'] # Se le asigna al mensaje la salida
                
        if i in range(0,180):        
                
                ang_left.input['distancia'] = self.distances[i]
                ang_left.compute() # Se operan los valores obtenidos
                angular2=ang_left.output['gira'] # Se le asigna al mensaje la salida


        self.moveObject.move(0.01,linear2,angular2)
        
    

    
    def main(self):
        rospy.spin()

               

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()
