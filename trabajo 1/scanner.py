#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data



class turtlebotScan(Node):
    def __init__(self):
        super().__init__('turtlebotScanObject')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.scanRanges = []
        self.linearVelocity = 0.5  # unit: m/s
        self.angularVelocity = 0.0  # unit: m/s        
        self.initScanState = False  # To get the initial scan data at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scanCallback, qos_profile=qos_profile_sensor_data)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.updateTimer = self.create_timer(0.010, self.updateCallback)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scanCallback(self, msg):
        self.scanRanges = msg.ranges
        self.initScanState = True
    
        print(self.scanRanges[0])

    def updateCallback(self):
        if self.initScanState is True:
            self.controlVel()

    def controlVel(self):
        twist = Twist()

        distance0 = ctrl.Antecedent(np.arange(0, 2, 0.11), 'distance0')

        distance45 = ctrl.Antecedent(np.arange(0, 2, 0.11), 'distance45')

        distance90 = ctrl.Antecedent(np.arange(0, 2, 0.11), 'distance90')

        distancem90 = ctrl.Antecedent(np.arange(0, 2, 0.11), 'distancem90')

        distancem45 = ctrl.Antecedent(np.arange(0, 1, 0.11), 'distancem45')

        angularvelocity= ctrl.Consequent(np.arange(0, 6, 0.5), 'angular')

        linearvelocity= ctrl.Consequent(np.arange(0, 2, 0.01), 'linear')

        distance0.automf(3)

        distance45.automf(3)

        distance90.automf(3)

        distancem90.automf(3)

        distancem45.automf(3)

        distancem90.automf(3)
        


        angularvelocity['Lejos'] = fuzz.trimf(angularvelocity.universe, [0, 0, 0])
        angularvelocity['normal'] = fuzz.trimf(angularvelocity.universe, [0, 3, 5])
        angularvelocity['parada'] = fuzz.trimf(angularvelocity.universe, [3, 5, 5])

        linearvelocity['paradal'] = fuzz.trimf(linearvelocity.universe, [0, 0, 0])
        linearvelocity['normall'] = fuzz.trimf(linearvelocity.universe, [0, 0.5, 1])
        linearvelocity['Lejosl'] = fuzz.trimf(linearvelocity.universe, [0.5, 1, 1])

        rule1 = ctrl.Rule(distance0['good'] & distance45['good'] | distance90['good'] & distancem90['good'] & distancem45['good'], angularvelocity['Lejos'])
        rule2 = ctrl.Rule(distance0['average'] | distance45['average'] | distance90['average'] | distancem90['average'] | distancem45['average'], angularvelocity['normal'])
        rule3 = ctrl.Rule(distance0['poor'] | distance45['poor'] | distance90['poor'] | distancem90['poor'] | distancem45['poor'], angularvelocity['parada'])
        rule4 = ctrl.Rule(distance0['good'] & distance45['good'] & distance90['good'] & distancem90['good'] & distancem45['good'], linearvelocity['Lejosl'])
        rule5 = ctrl.Rule(distance0['average'] | distance45['average'] | distance90['average'] | distancem90['average'] | distancem45['average'], linearvelocity['normall'])
        rule6 = ctrl.Rule(distance0['poor'] | distance45['poor'] | distance90['poor'] | distancem90['poor'] | distancem45['poor'], linearvelocity['paradal'])

        angularvelocity_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        linearvelocity_ctrl = ctrl.ControlSystem([rule4, rule5, rule6])

        Angularvelocity = ctrl.ControlSystemSimulation(angularvelocity_ctrl)
        Linearvelocity = ctrl.ControlSystemSimulation(linearvelocity_ctrl)
        Angularvelocity.input['distance0'] = self.scanRanges[0:359]
        Linearvelocity.input['distance0'] = self.scanRanges[0]
        Angularvelocity.input['distance45'] = self.scanRanges[45]
        Linearvelocity.input['distance45'] = self.scanRanges[45]
        Angularvelocity.input['distance90'] = self.scanRanges[90]
        Linearvelocity.input['distance90'] = self.scanRanges[90]
        Angularvelocity.input['distancem90'] = self.scanRanges[-90]
        Linearvelocity.input['distancem90'] = self.scanRanges[-90]
        Angularvelocity.input['distancem45'] = self.scanRanges[-45]
        Linearvelocity.input['distancem45'] = self.scanRanges[-45]

        Angularvelocity.compute()
        Linearvelocity.compute()
        
        print(angular_velocity.output['angular'])
  
        #print(self.scanRanges[0])
        self.cmdVelPub.publish(twist)

def main(args=None):
    print('start')
    rclpy.init(args=args)
    turtlebotScanObject = turtlebotScan()
    rclpy.spin(turtlebotScanObject)

    turtlebotScanObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

