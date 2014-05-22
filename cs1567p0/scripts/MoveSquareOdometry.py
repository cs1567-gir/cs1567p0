#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

edge = False
turned = False
n_edges = 0
currently_odom = False

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
    if(data.pose.pose.position.x > 0.75*x and data.pose.pose.position.x < 0.98*x):
        command.linear.x = 0.2
        send_command(command)
    elif(data.pose.pose.position.x < 0.97*x):
        command.linear.x = 0.4
        send_command(command)
    elif(data.pose.pose.position.x > 1.02*x):
        command.linear.x = -0.1
        send_command(command)
    else:
        command.linear.x = 0.0
        send_command(command)
        return True

    return False

def rotate(target_theta, data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    current_theta = get_angle(data)
    if abs(target_theta - current_theta) > 1.0:
        # get error between current and target
        if (target_theta - current_theta) < 0:
            command.angular.z = min((target_theta - current_theta)/180, -0.2)
        else:
            command.angular.z = max((target_theta - current_theta)/180, 0.2)
    else:
        command.angular.z = 0.0
        send_command(command)
        return True
    send_command(command)
    return False

# performed everytime the kobuki robot gives us pose data
def odometry_callback(data):
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    global edge
    global n_edges
    global turned
    global currently_odom

    if not currently_odom:
        currently_odom = True
        if not edge:
            print "moving forward"
            edge = move(1.0, data)    
        elif not turned:
            print "rotating"
            turned = rotate(-90.0, data)
        else:
            print "finished rotation, resetting odometry"
            while pub.get_num_connections() < 1:
                rospy.sleep(0.1)
            pub.publish(Empty())
            n_edges += 1
            edge = False
            turned = False
            rospy.sleep(0.5)
        currently_odom = False


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
