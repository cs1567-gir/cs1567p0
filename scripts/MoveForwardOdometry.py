#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

odom = None
run = False

# get current yaw angle from the supplied quaternion
def get_angle(data):
    w = data.pose.pose.orientation.w
    z = data.pose.pose.orientation.z
    mag = math.sqrt(w**2 + z**2)
    w /= mag
    z /= mag
    angle = math.copysign(2*math.acos(w), z)
    return 180.0/math.pi*angle

def move(x, data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    if(odom.pose.pose.position.x > 0.75*x and odom.pose.pose.position.x < 0.99*x):
        command.linear.x = 0.2
        send_command(command)
    elif(odom.pose.pose.position.x < 0.99*x):
        command.linear.x = 0.5
        send_command(command)
    elif(odom.pose.pose.position.x > 1.01*x):
        command.linear.x = -0.1
        send_command(command)
    else:
        command.linear.x = 0.0
        send_command(command)

def keep_printing():
    global run
    run = True
    x = 0
    while True:
        print x
        x += 1
        rospy.sleep(1)

# performed everytime the kobuki robot gives us pose data
def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    global odom
    odom = data
    move(1.0, data)
    if not run:
        keep_printing()
    print "callback" 

# initialize the ros node and its communications
def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveForwardOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    
# if this is run as a standalone
if __name__ == "__main__":
    initialize()
