#!/usr/bin/env python
import rospy
import logging
from geometry_msgs.msg import Twist
from cs1567p0.srv import *

def move_square():
    rospy.init_node('MoveSquareNoOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    command = Twist()
    e_t = 4.95
    r_t = 4.15

    for i in range(4):
        try:
            send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
            command.linear.x = 0.25
            response = send_command(command)
            rospy.sleep(e_t) # edge 1
            command.linear.x = 0.0
            command.angular.z = -0.5
            response = send_command(command)
            rospy.sleep(r_t) # corner 1
            command.angular.z = 0.0
            command.linear.x = 0.0
            response = send_command(command)
            rospy.sleep(0.25)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == "__main__":
    move_square()
