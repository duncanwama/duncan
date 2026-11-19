#!/usr/bin/env python3
# auto_drive.py - drive the chassis through waypoints and park in the green bay

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ---------------- Route (metres, odom frame) ----------------
# The robot must be spawned at (0, 0) facing +x.
WAYPOINTS = [
    (1.0, -0.6),   # 1: move right of box 1
    (2.0, -0.6),   # 2: drive past box 1
    (3.0,  0.0),   # 3: pass under box 2
    (4.5,  0.0),   # 4: pass over box 3
    (6.0,  0.0),   # 5: centre of the green bay
]

# ---------------- Tuning values ----------------
MAX_LINEAR      = 0.3    # top forward speed (m/s)
MAX_ANGULAR     = 1.0    # top turning speed (rad/s)
K_LINEAR        = 0.5    # forward speed = K_LINEAR x distance
K_ANGULAR       = 2.0    # turning speed = K_ANGULAR x angle error
TURN_FIRST      = 0.3    # if pointing more than this far off (rad), turn on the spot
TOLERANCE       = 0.10   # close enough to a waypoint (m)
FINAL_TOLERANCE = 0.05   # stop more accurately in the bay (m)


def clamp(value, limit):
    """Keep value between -limit and +limit."""
    return max(-limit, min(limit, value))


class AutoDriver:
    def __init__(self):
        rospy.init_node('auto_drive')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.x = None
        self.y = None
        self.yaw = None
        rospy.on_shutdown(self.stop)          # stop the robot when you press Ctrl+C

    def odom_callback(self, msg):
        """Called about 50 times a second with the robot's latest position."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation         # heading is stored as a quaternion
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def stop(self):
        self.cmd_pub.publish(Twist())         # a Twist of all zeros means stop

    def go_to(self, goal_x, goal_y, tolerance):
        """Drive to one point using feedback from /odom."""
        rate = rospy.Rate(10)                 # 10 corrections per second
        cmd = Twist()
        while not rospy.is_shutdown():
            # 1. Read and compare
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.hypot(dx, dy)
            if distance < tolerance:
                self.stop()
                return
            target_angle = math.atan2(dy, dx)
            # angle error kept between -pi and +pi, so the robot takes the short way round
            angle_error = math.atan2(math.sin(target_angle - self.yaw),
                                     math.cos(target_angle - self.yaw))

            # 2. Act
            cmd.angular.z = clamp(K_ANGULAR * angle_error, MAX_ANGULAR)
            if abs(angle_error) > TURN_FIRST:
                cmd.linear.x = 0.0            # pointing the wrong way: turn first
            else:
                cmd.linear.x = min(K_LINEAR * distance, MAX_LINEAR)
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def run(self):
        rospy.loginfo('Waiting for /odom ...')
        while self.x is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        for number, (wx, wy) in enumerate(WAYPOINTS, start=1):
            last = (number == len(WAYPOINTS))
            rospy.loginfo('Going to waypoint %d: (%.1f, %.1f)', number, wx, wy)
            self.go_to(wx, wy, FINAL_TOLERANCE if last else TOLERANCE)

        rospy.loginfo('Arrived in the green bay at (%.2f, %.2f)', self.x, self.y)


if __name__ == '__main__':
    try:
        AutoDriver().run()
    except rospy.ROSInterruptException:
        pass
