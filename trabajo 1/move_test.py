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
        # dist = self.distances[0]
        # print(dist)
        

        # distance_sensor = ctrl.Antecedent(np.arange(0.5, 2.2, 0.1), 'd_sensor')
        # angular_velocity = ctrl.Consequent(np.arange(0.0, 0.4, 0.1), 'angular')
        
        # distance_sensor.automf(3)

        # angular_velocity['cerca'] = fuzz.trimf(angular_velocity.universe, [0.0, 0.0, 0.2])
        # angular_velocity['normal'] = fuzz.trimf(angular_velocity.universe, [0.0, 0.2, 0.4])
        # angular_velocity['lejos'] = fuzz.trimf(angular_velocity.universe, [0.2, 0.4, 0.4])

        # rule1 = ctrl.Rule(distance_sensor['poor'] , angular_velocity['lejos'])
        # rule2 = ctrl.Rule(distance_sensor['average'], angular_velocity['normal'])
        # rule3 = ctrl.Rule(distance_sensor['good'] , angular_velocity['cerca'])

        # angular_velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        # angular_velocity_c = ctrl.ControlSystemSimulation(angular_velocity_ctrl)

        #angular_velocity_c.input['d_sensor'] = self.distances[0]
        # angular_velocity_c.compute()

        # print("angular")
        # print(angular_velocity_c.output['angular'])
        # print("distancia")
        # print(self.distances[0])
        
        # angular = angular_velocity_c.output['angular']
        
        # for n in np.arange(-90, 135, 45):
        #     print("=====================================")
        #     print("Angle: " + str(n) + " - distance: " + str(self.distances[n]))
        #     angular_velocity_c.input['d_sensor'] = self.distances[n]
        #     angular_velocity_c.compute()
        #     print("calculo: " + str(angular_velocity_c.output['angular']))
        
        # if i in rp.arange(-90, 90, 45):
        #     tipp.input['distancia'] = self.distances[i]
        #     tipp.compute() # Se operan los valores obtenidos
        #     angular2 =tipp.output['gira'] # Se le asigna al mensaje la salida

        # if(angular < 0.4):
        #     self.moveObject.move(0.3,0.0,angular)
        # angular2 = Angularvelocity.output['angular']
        # linear2 = Linearvelocity.output['linear']
        # self.moveObject.move(0.3,linear2,angular2)

        sensor_distance = ctrl.Antecedent(np.arange(0.5, 2.2, 0.1), 'd_sensor')
        linear_velocity = ctrl.Consequent(np.arange(-1.0, 1.0, 0.1), 'linear_velocity') #Salida de la velocidad lineal
        velocity_angular = ctrl.Consequent(np.arange(0, 1.5, 0.3),'gira')

        sensor_distance.automf(3)

        linear_velocity['Freno'] = fuzz.trimf(linear_velocity.universe,[-1.0, -1.0, 0.0]) # En ese rango de datos debe frenar
        linear_velocity['Medio'] = fuzz.trimf(linear_velocity.universe,[-1.0, 0.0, 1]) # En ese rango de datos debe moverse poco
        linear_velocity['Rapido'] = fuzz.trimf(linear_velocity.universe,[0.0, 1.0, 1.0]) 

        rule1 = ctrl.Rule(sensor_distance['poor'], linear_velocity['Freno'])
        rule2 = ctrl.Rule(sensor_distance['average'], linear_velocity['Medio'])
        rule3 = ctrl.Rule(sensor_distance['good'], linear_velocity['Rapido'])

        linear_velocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        linear_velocity_c = ctrl.ControlSystemSimulation(linear_velocity_ctrl)

        # linear_velocity_c.input['d_sensor'] = self.distances[0]
        # linear_velocity_c.compute()

        # print("Angle: 0  - distance: " + str(self.distances[0]))
        # print(linear_velocity_c.output['linear_velocity'])      
        # velocity = linear_velocity_c.output['linear_velocity']

        velocity = 0
        # for n in np.arange(0, 360, 90):
        #     print("=====================================")
        #     print("Angle: " + str(n) + " - distance: " + str(self.distances[n]))
            
        #     linear_velocity_c.input['d_sensor'] = self.distances[n]
        #     linear_velocity_c.compute()
            
        #     print("calculo: " + str(linear_velocity_c.output['linear_velocity']))
            
        #     velocity = linear_velocity_c.output['linear_velocity']
        #     self.moveObject.move(0.1, velocity, 0.0)

        print("Angle: 0 - distance: " + str(self.distances[0]))
        print("Angle: -180 - distance: " + str(self.distances[-180]))
        
        if self.distances[0] > self.distances[-180]:
            linear_velocity_c.input['d_sensor'] = self.distances[0]
        else:
            linear_velocity_c.input['d_sensor'] = self.distances[-180]
        
        linear_velocity_c.compute()
        velocity = linear_velocity_c.output['linear_velocity']
        
        print(velocity) 
        # self.moveObject.move(0.01, velocity, 0.0)

        # for i in [-180, 0]:
        #     print("Angle: " + str(i) + " - distance: " + str(self.distances[i]))
        #     linear_velocity_c.input['d_sensor'] = self.distances[i]
        #     linear_velocity_c.compute()
        #     velocity = linear_velocity_c.output['linear_velocity']
        #     if(i < 0):
        #         velocity = -velocity
        #     print(velocity) # Se imprime el valor de salida
        #     self.moveObject.move(0.01, velocity, 0.0)

        # self.moveObject.move(0.01, velocity, 0.0)
        


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()