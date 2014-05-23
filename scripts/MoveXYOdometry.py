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
    angle = math.copysign(2*math.acos(w), z)
    return 180.0/math.pi*angle

# sets linear and angular values to get to desired location (x,y)
# based on position and orientation information
def move(x,y):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    current_x = odom.pose.pose.position.x
    current_y = odom.pose.pose.position.y
    # get current rotation
    current_theta = get_angle(odom)
    if current_theta < 0.0:
        current_theta += 360.0

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
        command.linear.x = 0.4

    # NOTE: we do not send the command until we have checked the angle value
    # this prevents the robot from starting to move if the angle is too far off

    # build direction vector from this information
    
    # (now we have target rotation angle)
    target_theta = 180/math.pi * math.asin((y - current_y)/distance)
    if (x - current_x) < 0.0:
        target_theta = 180/math.pi * math.acos((x - current_x)/distance)
        if (y - current_y) < 0.0:
            target_theta = 360 - target_theta

    if target_theta < 0:
        print "pre-correction target: ", target_theta
        target_theta += 360
        print "post-correction target: ", target_theta

    # target_theta = math.copysign(target_theta, x - current_x)

    # if desired point is in quadrant 2 or 3 as compared to current point, compensate for asin's limitations
    error = target_theta - current_theta
    print "error: ", error
    if error > 180:
        error -= 360
    if error < -180:
        error += 360
    print "compensated error: ", error 
    
    # if our angle is off from the target by more than some error value
    if abs(error) > 0.5:
        # if we are really far from the target angle, stop forward motion before correcting
        if abs(error) > 10.0:
            command.linear.x = 0.0
        # set angular velocity to rotate towards destination (scale by how far from desired angle we are)
        # command.angular.z = max((target_theta - current_theta)/180, 0.2)
        print "target_theta: ", target_theta, "current_theta: ", current_theta
        if(error < 0):
            print "turning clockwise"
            command.angular.z = min((error)/180, -0.3)
        else:
            print "turning counterclockwise"
            command.angular.z = max((error)/180, 0.3)
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
    
    # corner[0] = True
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
