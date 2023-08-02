#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16

rospy.init_node('steer_test_node', anonymous = True)

pub = rospy.Publisher("Car_Control_cmd/SteerAngle_Int16", Int16, queue_size=1)

steer = 0
old_steer = 0

while 1:
    # global steer, old_steer
    steer = int(input("input : "))
    if (abs(old_steer - steer) <= 2):
        steer = old_steer
        pub.publish(steer)
        print(steer, old_steer)
    else:    
        pub.publish(steer)
        print(steer, old_steer)
    old_steer = steer
rospy.spin()