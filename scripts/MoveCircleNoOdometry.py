#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *

def move_circle():
    rospy.init_node('MoveCircleNoOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    command = Twist()
    
    omega = 0.6
    velocity = 0.3
    # 10.47 calculated
    drive_time = 11.7

    try:
        send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
        command.linear.x = velocity
        command.angular.z = omega
        response = send_command(command)
        rospy.sleep(drive_time)
        command.linear.x = 0.0
        command.angular.z = 0.0
        response = send_command(command)
        print response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    move_circle()
