#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

odom = None
corner = [False, False, False, False]

# get current yaw angle from the supplied quaternion
def get_angle(data):
    w = data.pose.pose.orientation.w
    z = data.pose.pose.orientation.z
    mag = math.sqrt(w**2 + z**2)
    w /= mag
    z /= mag
    angle = 2*math.acos(w)
    return 180.0/math.pi*angle

# sets linear and angular values to get to desired location (x,y)
# based on position and orientation information
def move(x,y):
	current_x = odom.data.data.position.x
	current_y = odom.data.data.position.y
	# get current rotation
	current_theta = get_angle(odom)
    distance = math.sqrt((current_x - x)**2 + (current_y - y)**2)

    # first, check if we are within an acceptible radius of our desired point
    # this is risky, should probably increase the OK radius
    if distance < 0.02:   # 2 cm dead zone
    	command.linear.x = 0.0
    	command.angular.x = 0.0
    	send_command(command)
    	return True
    elif distance < 0.25: # if we are closer than 25 cm, slow down
        command.linear.x = 0.2
    else:
    	command.linear.x = 0.5

    # NOTE: we do not send the command until we have checked the angle value
    # this prevents the robot from starting to move if the angle is too far off

    # build direction vector from this information
    current_d = vector(math.cos(current_theta), math.sin(current_theta), 0)
    # determine unit direction vector to destination
    dest_d = vector(x - current_x, y - current_y, 0).norm
    # (now we have target rotation angle)
    target_theta = 180/math.pi * math.asin(dest_d.y)
    # if our angle is off from the target by more than some error value
    if abs(current_theta - target_theta) > 0.1:
        # if we are really far from the target angle, stop forward motion before correcting
    	if abs(current_theta - target_theta) > 2.0:
            command.linear.x = 0.0
        # set angular velocity to rotate towards destination (scale by how far from desired angle we are)
        command.angular.z = max((target_theta - current_theta)/180, 0.2)
    
    send_command(command)
    return False
'''
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
'''

# performed everytime the kobuki robot gives us pose data
def odometry_callback(data):
    global odom
    global corner
    odom = data
    
    if not corner[0]:
        corner[0] = move(1.0, 0.0)
    '''
    elif not corner[1]:
        corner[1] = move(1.0, 1.0)
    elif not corner[2]:
        corner[2] = move(0.0, 1.0)
    elif not corner[3]:
    	corner[3] = move(0.0, 0.0)
    '''
    else:
        # stop all motion
 


def move_square():
    move(1,0)
    move(1,1)
    move(0,1)
    move(0,0)

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
