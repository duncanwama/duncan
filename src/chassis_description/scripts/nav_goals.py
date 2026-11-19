#!/usr/bin/env python3
# nav_goals.py - send the chassis to a list of goals with move_base (Module 3.5.2)

import math
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# ---------------- Goals (map frame: metres, and degrees from +x) ----------------
GOALS = [
    ('A: south-east corner',  3.0, -2.0,  90),
    ('B: behind box 1',       1.0,  2.3, 180),
    ('C: north-west nook',   -3.0,  2.2, -90),
    ('D: south-west corner', -3.0, -2.2,   0),
    ('Home',                  0.0,  0.0,   0),
]

TIME_LIMIT = 120.0   # seconds allowed for one goal


def make_goal(x, y, yaw_deg):
    """A move_base goal: a position and heading in the map frame."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, math.radians(yaw_deg))   # heading -> quaternion
    o = goal.target_pose.pose.orientation
    o.x, o.y, o.z, o.w = q
    return goal


def main():
    rospy.init_node('nav_goals')
    # 'move_base' with no leading / : in /chassis this becomes /chassis/move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('Waiting for move_base ...')
    client.wait_for_server()

    for name, x, y, yaw in GOALS:
        rospy.loginfo('Going to %s (%.1f, %.1f)', name, x, y)
        client.send_goal(make_goal(x, y, yaw))

        finished = client.wait_for_result(rospy.Duration(TIME_LIMIT))
        if not finished:
            client.cancel_goal()
            rospy.logwarn('%s: took longer than %.0f s, cancelled', name, TIME_LIMIT)
        elif client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('%s: reached', name)
        else:
            rospy.logwarn('%s: failed (state %d)', name, client.get_state())

    rospy.loginfo('All goals done')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
