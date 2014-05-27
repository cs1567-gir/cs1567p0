#!/usr/bin/env python
import rospy, math
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *
from datetime import datetime

class KobukiState:
    def __init__(self):
        self.time = datetime.now().microsecond
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.distance = 0.0
        self.total_distance = 0.0
        self.velocity = 0.0

odom = None
corner = [False, False, False, False, False]
last_state = KobukiState()


# get current yaw angle from the supplied quaternion
def get_angle(data):
    w = data.pose.pose.orientation.w
    z = data.pose.pose.orientation.z
    mag = math.sqrt(w**2 + z**2)
    w /= mag
    z /= mag
    angle = math.copysign(2*math.acos(w), z)
    return 180.0/math.pi*angle

# performed everytime the kobuki robot gives us pose data
def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global last_state
    global odom
    global corner

    c_t = datetime.now().microsecond
    delta_t = c_t - last_state.time
    delta_t *= 1000000.0
    delta_x = data.pose.pose.position.x - last_state.x
    delta_y = data.pose.pose.position.y - last_state.y
    heading = get_angle(data)
    if heading < 0:
        heading += 360
    delta_h = heading - last_state.heading
    distance = math.sqrt(delta_x**2 + delta_y**2)
    velocity = distance / delta_t # approx speed
    avg_velocity = 0.7 * velocity + 0.3 * last_state.velocity    

    # update global state
    last_state.time = c_t
    last_state.x = data.pose.pose.position.x
    last_state.y = data.pose.pose.position.y
    last_state.heading = heading
    last_state.velocity = avg_velocity
    last_state.distance = distance
    #last_state.total_distance += 0.7 * distance + (0.3 * (avg_velocity * delta_t))
    last_state.total_distance += distance
  
    r = 0.5
    s = last_state.total_distance
    theta = (s/r)
    theta = 180/math.pi * theta

    if last_state.total_distance < 0.99 * (math.pi * r * 2):
        # fix the angle
        error = theta - heading
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0

        if abs(error) > 0.2:
            if error < 0:
                command.angular.z = min(error*0.05, -0.3)
            elif error > 0:
                command.angular.z = max(error*0.05, 0.3)
            else:
                command.angular.z = 0.0
        command.linear.x = 0.2
    elif last_state.total_distance < 0.995 * (math.pi * r * 4):
        theta = -theta
        if theta < 0:
            theta += 360.0
        error = theta - heading
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0

        if abs(error) > 0.2:
            if error < 0:
                command.angular.z = min(error*0.05, -0.3)
            elif error > 0:
                command.angular.z = max(error*0.05, 0.3)
            else:
                command.angular.z = 0.0
        command.linear.x = 0.2        
    else:
        command.angular.z = 0.0
        command.linear.x = 0.0
    send_command(command)
    print "total distance: ", last_state.total_distance

# initialize the ros node and its communications
def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveInfinityOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    
# if this is run as a standalone
if __name__ == "__main__":
    initialize()
 
