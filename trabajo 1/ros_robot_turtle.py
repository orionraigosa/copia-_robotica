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

class moveRobot(object):
    def __init__(self):
        self.ctrl_c = False 
        # creacion de publicacion
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

    #funcion de publicación de datos
    def publishOnceCmdVel(self, cmd):
        while not self.ctrl_c:
            connections = self.velPublisher.get_num_connections()
            if connections > 0:
                self.velPublisher.publish(cmd)
                break
            else:
                self.rate.sleep()
    
    #funcion para mover robot
    def move(self, movingTime, linear1, angular1): 
        cmd = Twist()
        cmd.linear.x = linear1
        cmd.angular.z = angular1
        
        self.publishOnceCmdVel(cmd)
        time.sleep(movingTime)

class scanner:
    def __init__(self):
        self.moveObject = moveRobot()
        self.initScan()
        self.laserSub = rospy.Subscriber("/scan",LaserScan,self.scanCallback)

    #funcion para scanner
    def initScan(self):
        self._scan = None
        while self._scan is None: 
            try:        
                #obtiene el mensaje enviado por el sensor        
                self._scan = rospy.wait_for_message("/scan", LaserScan, timeout=1) 
                return True
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying")
        
        rospy.loginfo("/scan topic successfully") 

    def scanCallback(self,msg):
        self.distances = msg.ranges

        #Entrada de medicion sensor
        distancia = ctrl.Antecedent(np.arange(0.5,2.5,0.5), 'distancia')
        
        #definicion de salida para velocidad lineal
        velocidad = ctrl.Consequent(np.arange(-1.0, 1.5, 0.5), 'velocidad')
        # definicion de salida para velocidad angular 
        gira = ctrl.Consequent(np.arange(-1.5, 1.5, 0.3),'gira') 

        #membresias y universos para medicion de sensor de entrada
        distancia['Muycerca'] = fuzz.trimf(distancia.universe,[0.2,0.5,0.7]) 
        distancia['cerca'] = fuzz.trimf(distancia.universe,[0.5,0.7,3.0]) 
        distancia['lejos'] = fuzz.trimf(distancia.universe,[0.7,3.0,3.0]) 
        
        #membresias y universos para salida de velocidad lineal
        velocidad['reversa'] = fuzz.trimf(velocidad.universe,[-1.0,-1.0,0.0]) 
        velocidad['Medio'] = fuzz.trimf(velocidad.universe,[-1.0,1.2,1.5]) 
        velocidad['Rapido'] = fuzz.trimf(velocidad.universe,[1.2,1.5,1.5]) 
	
        #membresias y universos para salida de velociad angular
        gira['right'] = fuzz.trimf(gira.universe,[-1.5,-1,-0.5])
        gira['slow'] = fuzz.trimf(gira.universe,[-1,-0.5,0.5]) 
        gira['left'] = fuzz.trimf(gira.universe,[0.5,1,1.5]) 

        # Creación de las reglas de inferencia o decisiones del robot autónomo
        rule_1 = ctrl.Rule(distancia['Muycerca'], velocidad['reversa'])
        rule_2 = ctrl.Rule(distancia['cerca'], velocidad['Medio'])
        rule_3 = ctrl.Rule(distancia['lejos'], velocidad['Rapido'])
        
        rule_4 = ctrl.Rule(distancia['lejos'], gira['slow'])
        rule_5 = ctrl.Rule(distancia['Muycerca'], gira['right'])
        rule_6 = ctrl.Rule(distancia['Muycerca'], gira['left'])
    
        #Control y simulacion de velocidad lineal
        vel_lineal = ctrl.ControlSystem([rule_1, rule_2, rule_3])
        vel = ctrl.ControlSystemSimulation(vel_lineal)

        # se aplican calculos a angulos seleccionados para obtener la velocidad lineal
        for i in range(-11, 11):
            vel.input['distancia'] = self.distances[i]
            # se ejecuta el proceso para obtener datos
            vel.compute()
            # Se asigna el valor obtenido a la variable
            velocidad_lineal = vel.output['velocidad'] 


        if i in range(0,-45):
            vel.input['distancia'] = self.distances[i]
            # se ejecuta el proceso para obtener datos
            vel.compute()
            # Se asigna el valor obtenido a la variable
            velocidad_lineal = vel.output['velocidad'] 

        if i in range(0,40):
            vel.input['distancia'] = self.distances[i]
            # se ejecuta el proceso para obtener datos
            vel.compute()
            # Se asigna el valor obtenido a la variable
            velocidad_lineal = vel.output['velocidad'] 

        if i in range(0,-90):
            vel.input['distancia'] = self.distances[i]
            # se ejecuta el proceso para obtener datos
            vel.compute()
            # Se asigna el valor obtenido a la variable
            velocidad_lineal = vel.output['velocidad'] 

        if i in range(0,90):
            vel.input['distancia'] = self.distances[i]
            # se ejecuta el proceso para obtener datos
            vel.compute()
            # Se asigna el valor obtenido a la variable
            velocidad_lineal = vel.output['velocidad'] 
                
        if i in range(-180,180):
                vel.input['distancia'] = self.distances[i]
                # se ejecuta el proceso para obtener datos
                vel.compute()
                # Se asigna el valor obtenido a la variable
                velocidad_lineal = vel.output['velocidad'] 
                
        # Control y simulacion de velocidad angular
        ang_right_ctrl = ctrl.ControlSystem([rule_4, rule_5])
        ang_right = ctrl.ControlSystemSimulation(ang_right_ctrl)
        
        ang_left_ctrl = ctrl.ControlSystem([rule_4, rule_6])
        ang_left = ctrl.ControlSystemSimulation(ang_left_ctrl)
    
        # se aplican calculos a angulos seleccionados para obtener la velocidad angular
        for i in range(-60,-30):
        	ang_right.input['distancia'] = self.distances[i]
        	ang_right.compute()
        	velocidad_angular = ang_right.output['gira'] 
    
        
        if i in range(30,60):     	                
            ang_left.input['distancia'] = self.distances[i]
             # Se realiza operación para obtner valores
            ang_left.compute()
            # se asigna el valor
            velocidad_angular = ang_left.output['gira']
                          
        if i in range(0,-180):        
        	ang_right.input['distancia'] = self.distances[i]
        	ang_right.compute() 
        	velocidad_angular = ang_right.output['gira'] 
                
        if i in range(0,180):        
            ang_left.input['distancia'] = self.distances[i]
            ang_left.compute() 
            velocidad_angular = ang_left.output['gira'] 

        print("Lineal_Velocity: " + str(velocidad_lineal))   
        print("Angular_Velocity: " + str(velocidad_angular)) 

        self.moveObject.move(0.01, velocidad_lineal, velocidad_angular)
        
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()
