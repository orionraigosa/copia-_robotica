import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print("====================================")
    

    print("s1 (45)")
    print(msg.ranges[45])

    print("s0 (0)")
    print(msg.ranges[0])

    print("s2 (45)")
    print(msg.ranges[-45])

rospy.init_node('laser_data')
sub = rospy.Subscriber('/scan',LaserScan, callback)
rospy.spin()