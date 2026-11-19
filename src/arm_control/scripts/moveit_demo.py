#!/usr/bin/env python3
# moveit_demo.py - MoveIt from Python: the table, named, joint and pose goals, a straight line

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped

NS = '/arm'                       # move_group runs in this namespace
DESC = '/arm/robot_description'


def down_pose(x, y, z):
    """A gripper pose at (x, y, z) with the fingers pointing straight down."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0.0      # 180 degrees about y:
    pose.orientation.y = 1.0      # ee_link's z axis points down
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_demo')

    robot = moveit_commander.RobotCommander(robot_description=DESC, ns=NS)
    scene = moveit_commander.PlanningSceneInterface(ns=NS)
    arm = moveit_commander.MoveGroupCommander('arm', robot_description=DESC, ns=NS)
    gripper = moveit_commander.MoveGroupCommander('gripper', robot_description=DESC, ns=NS)

    arm.set_max_velocity_scaling_factor(0.5)       # half speed
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_planning_time(5.0)
    arm.set_num_planning_attempts(5)

    rospy.loginfo('Planning frame: %s', arm.get_planning_frame())
    rospy.loginfo('End effector:   %s', arm.get_end_effector_link())
    rospy.loginfo('Groups:         %s', robot.get_group_names())

    # 1. Tell MoveIt about the table (same size and place as in the Gazebo world)
    rospy.sleep(1.0)                               # let the scene connection start
    table = PoseStamped()
    table.header.frame_id = 'world'
    table.pose.position.x = 0.375
    table.pose.position.z = 0.15
    table.pose.orientation.w = 1.0
    scene.add_box('table', table, size=(0.25, 0.40, 0.30))
    rospy.sleep(1.0)
    rospy.loginfo('Scene objects: %s', scene.get_known_object_names())

    # 2. Named poses
    arm.set_named_target('home')
    arm.go(wait=True)
    arm.stop()

    gripper.set_named_target('open')
    gripper.go(wait=True)
    gripper.stop()

    # 3. Joint goal: the 'ready' angles (Step 5)
    arm.go([0.0, 0.37, 0.62, 0.0, 2.15, 0.0], wait=True)
    arm.stop()

    # 4. Pose goal: 10 cm above the table, a little to the left of the cube
    arm.set_pose_target(down_pose(0.25, 0.10, 0.40))
    ok = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    rospy.loginfo('Pose goal %s', 'reached' if ok else 'FAILED')

    # 5. Cartesian path: straight down by 4 cm
    start = arm.get_current_pose().pose
    target = copy.deepcopy(start)
    target.position.z -= 0.04
    try:
        plan, fraction = arm.compute_cartesian_path([target], 0.01)        # newer MoveIt
    except TypeError:
        plan, fraction = arm.compute_cartesian_path([target], 0.01, 0.0)   # older MoveIt
    rospy.loginfo('Cartesian path: %.0f%% planned', fraction * 100)
    if fraction > 0.95:                            # only move if almost all of it was planned
        arm.execute(plan, wait=True)

    # 6. Back home
    arm.set_named_target('home')
    arm.go(wait=True)
    arm.stop()

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
