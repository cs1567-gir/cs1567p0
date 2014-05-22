#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

odom = None
started = False

def get_angle(data):
    w = data.pose.pose.orientation.w
    z = data.pose.pose.orientation.z
    mag = math.sqrt(w**2 + z**2)
    w/=mag
    z/=mag
    angle = 2*math.acos(w)
    return 180.0/math.pi*angle

def move(x,y):
    # get current rotation
    # build direction vector from rotation
    # determine direction vector to dest
    # (now we have target rotation angle)
    # rotate towards destination
    # drive to destination

    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    if(odom.pose.pose.position.x > 0.9*x and odom.pose.pose.position.x < x):
        command.linear.x = 0.2
        send_command(command)
    elif(odom.pose.pose.position.x < 0.99*x):
        command.linear.x = 0.5
        send_command(command)
    elif(odom.pose.pose.position.x > 1.05*x):
        command.linear.x = -0.1
        send_command(command)
    else:
        command.linear.x = 0.0
        send_command(command)


def odometry_callback(data):
    global odom
    global started
    odom=data
    if not started:
        started = True
        move(2,0)

def move_square():
    move(1,0)
    move(1,1)
    move(0,1)
    move(0,0)

def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveForwardOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    

if __name__ == "__main__":
    initialize()
