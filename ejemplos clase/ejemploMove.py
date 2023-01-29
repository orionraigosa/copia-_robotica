#!/usr/bin/python3

import sys
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

    def stop(self): # Se crea una funcion de detenido
        #rospy.loginfo("shutdown time! Stop the robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publishOnceCmdVel(cmd)

    def move(self, movingTime, linearSpeed, angularSpeed): # Se crea una funcion de movimiento
        cmd = Twist()
        cmd.linear.x = linearSpeed
        cmd.angular.z = angularSpeed
        
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
            except:
                rospy.loginfo("/scan topic is not ready yet, retrying") # Sino se tiene exito se espera a su lectura
        
        rospy.loginfo("/scan topic READY") # Se informa de su lectura

    def scanCallback(self,msg):
        distances = msg.ranges
        dist = distances[0]
        print(dist) 

        if dist > 3.4:
            self.moveObject.move(0.3,0.2,0.0)
            rospy.loginfo("Moving")
        elif dist < 1.0:
            self.moveObject.stop()
            rospy.loginfo("Stop")
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('myScan')
    node = scanner()

node.main()
