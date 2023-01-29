#!/usr/bin/python3
# coding=utf-8

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class readData(object): # Creo una clase para leer los datos
    def __init__(self): # Inicio el constructor
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

    def scanCallback(self,msg): # Se define una funcion para almacena el mensaje en una variable global de la clase
        self._scan = msg

    def getData(self): # Se define una funcion para la lectura de los datos
        return self._scan.ranges # La funcion al ser invocada retorna los datos que se encuentran dentro de ranges

class moveRobot(object): # Se crea una clase para realizar los movimientos del robot
    def __init__(self):
        self.scanObject = readData() # Se crea una variable global en la clase que invoque la la clase de lectura de datos
        self.velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Se crea el publicador
        self.ctrl_c = False # Se crea una variable de control
        rospy.on_shutdown(self.shutdownhook) # Se determina el estado de rospy
        self.rate = rospy.Rate(10) # Se define una velocidad de operacion

    def publishOnceCmdVel(self, cmd): # Se crea la funcion de publicacion
            while not self.ctrl_c:
                connections = self.velPublisher.get_num_connections()
                if connections > 0:
                    self.velPublisher.publish(cmd)
                    rospy.loginfo("Cmd Published")
                    break
                else:
                    self.rate.sleep()
    
    def shutdownhook(self): # Se crea la funcion de apagado
        self.stop()
        self.ctrl_c = True

    def stop(self): # Se crea una funcion de detenido
        rospy.loginfo("shutdown time! Stop the robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publishOnceCmdVel(cmd)

    def moveX(self, movingTime, linearSpeed, angularSpeed): # Se crea una funcion de movimiento
        cmd = Twist()
        cmd.linear.x = linearSpeed
        cmd.angular.z = angularSpeed
        
        rospy.loginfo("Moving Forwards")
        self.publishOnceCmdVel(cmd)
        time.sleep(movingTime)
        #self.stop()
        rospy.loginfo("######## Finished Moving Forwards")

    def move(self): # Se crea una funcion para la operacion
        distances = self.scanObject.getData() # Se almacena en una variable los datos leidos desde la clase de lectura de datos
        print(len(distances))
        print(distances)
        dist = distances[0]
        while dist > 0.5:
            distances = self.scanObject.getData()
            dist = distances[0]
            self.moveX(0.01,0.5,0.0)

if __name__ == '__main__': # Se inicia la operacion del codigo
    rospy.init_node('myScan', anonymous=True) # Se inicia el nodo
    moveObject = moveRobot() # Se invoca la clase de movimiento
    try:
        moveObject.move() # Se invoca la funcion de operacion que se encuentra dentro de la clase de movimiento
    except rospy.ROSInterruptException:
        pass
