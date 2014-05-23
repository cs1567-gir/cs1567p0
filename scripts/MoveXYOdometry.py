#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

odom = None
corner = [False, False, False, False, False]

# get current yaw angle from the supplied quaternion
def get_angle(data):
    w = data.pose.pose.orientation.w
    z = data.pose.pose.orientation.z
    mag = math.sqrt(w**2 + z**2)
    w /= mag
    z /= mag
    angle = math.copysign(2*math.acos(w), z)
    return 180.0/math.pi*angle

# sets linear and angular values to get to desired location (x,y)
# based on position and orientation information
def move(x,y):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    current_x = odom.pose.pose.position.x
    current_y = odom.pose.pose.position.y
    x_error = x - current_x
    y_error = y - current_y

    # get current rotation from our starting orientation
    current_theta = get_angle(odom)
    if current_theta < 0.0:
        current_theta += 360.0

    # current distance to target
    distance = math.sqrt(x_error**2 + y_error**2)

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
        command.linear.x = 0.4

    # NOTE: we do not send the command until we have checked the angle value
    # this prevents the robot from starting to move if the angle is too far off

    # determine target heading
    target_theta = 180/math.pi * math.asin(y_error/distance)
    
    # since asin only handles quadrants 1 and 4, we neet to use acos for others
    if x_error < 0.0:
        target_theta = 180/math.pi * math.acos(x_error/distance)
        if y_error < 0.0: # this still leaves quadrant 3 uncovered, so a manual compensation is necessary
            target_theta = 360 - target_theta

    # now shift target theta to be between 0 and 360
    if target_theta < 0:
        print "pre-correction target: ", target_theta
        target_theta += 360
        print "post-correction target: ", target_theta

    # determine the difference between our target heading and our current heading
    theta_error = target_theta - current_theta
    print "angle error: ", theta_error
    if theta_error > 180:
        theta_error -= 360
    if theta_error < -180:
        theta_error += 360
    print "compensated angle error: ", theta_error 
    
    # if our angle is off from the target by more than 0.5 degrees, we begin to compensate
    if abs(theta_error) > 0.5:
        # if we are really far from the target angle, stop forward motion and correct
        if abs(theta_error) > 6.0:
            command.linear.x = 0.0
        # set angular velocity to rotate towards destination (scaled by our current error)
        print "target_theta: ", target_theta, "current_theta: ", current_theta
        if(theta_error < 0):
            print "turning clockwise"
            command.angular.z = min((theta_error)/180, -0.3)
        else:
            print "turning counterclockwise"
            command.angular.z = max((theta_error)/180, 0.3)
    else:
        command.angular.z = 0.0

    send_command(command)
    return False

# performed everytime the kobuki robot gives us pose data
def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global odom
    global corner
    odom = data
    
    
    if not corner[0]:
        corner[0] = move(1.0, 0.6)
    elif not corner[1]:
        corner[1] = move(-0.2, 0.6)
    elif not corner[2]:
        corner[2] = move(0.8, 0.0)
    elif not corner[3]:
    	corner[3] = move(0.4, 1.0)
    elif not corner[4]:
    	corner[4] = move(0.0, 0.0)
    else:
        command.linear.x = 0.0
        command.angular.x = 0.0
        send_command(command)
        print "current_theta: ", get_angle(data) 
    
    '''    
    if not corner[0]:
        corner[0] = move(1.0, 0.0)
    elif not corner[1]:
        corner[1] = move(1.0, 1.0)
    elif not corner[2]:
        corner[2] = move(0.0, 1.0)
    elif not corner[3]:
    	corner[3] = move(0.0, 0.0)
    else:
        command.linear.x = 0.0
        command.angular.x = 0.0
        send_command(command)
        print "current_theta: ", get_angle(data) 
    '''
# initialize the ros node and its communications
def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveXYOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    
# if this is run as a standalone
if __name__ == "__main__":
    initialize()
