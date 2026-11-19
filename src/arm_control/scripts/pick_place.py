#!/usr/bin/env python3
# pick_place.py - pick the red cube from the table and place it 12 cm to the left

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

# ---------------- Names ----------------
NS = '/arm'                          # MoveIt runs in this namespace
DESC = '/arm/robot_description'
ARM_GROUP = 'arm'
GRIPPER_GROUP = 'gripper'            # the name you chose in the Setup Assistant
EE_LINK = 'ee_link'
TOUCH_LINKS = ['palm', 'left_finger', 'right_finger']   # may touch the held cube

# ---------------- Positions (m, world frame) ----------------
PICK_XY = (0.30, 0.00)               # where the cube starts
PLACE_XY = (0.30, 0.12)              # where it should end up
ABOVE_Z = 0.38                       # ee_link height above the cube
GRASP_Z = 0.33                       # ee_link height when holding the cube
CUBE_Z = 0.32                        # cube centre when it sits on the table
CUBE_SIZE = 0.04
HELD_SIZE = 0.036                    # slightly smaller copy for planning, so it is
                                     # not "touching" the table when lifted

# ---------------- Gripper (m per finger) ----------------
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_CUBE = [0.019, 0.019]        # light touch: the link attacher does the holding


def down_pose(x, y, z):
    """ee_link at (x, y, z), fingers pointing straight down."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y = 0.0, 1.0
    pose.orientation.z, pose.orientation.w = 0.0, 0.0
    return pose


class PickPlace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place')

        self.scene = moveit_commander.PlanningSceneInterface(ns=NS)
        self.arm = moveit_commander.MoveGroupCommander(ARM_GROUP, robot_description=DESC, ns=NS)
        self.gripper = moveit_commander.MoveGroupCommander(GRIPPER_GROUP, robot_description=DESC, ns=NS)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(5)

        rospy.wait_for_service('/link_attacher_node/attach')
        self.gz_attach = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.gz_detach = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        rospy.sleep(1.0)                 # let the planning scene connection start

    # ======================= Helpers =======================
    def check(self, ok, stage):
        """Stop the whole program if a stage failed."""
        if not ok:
            rospy.logerr('FAILED at: %s', stage)
            moveit_commander.roscpp_shutdown()
            sys.exit(1)
        rospy.loginfo('OK: %s', stage)

    def add_box(self, name, x, y, z, size):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.0
        self.scene.add_box(name, pose, size=size)

    def go_named(self, name):
        self.arm.set_named_target(name)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        return ok

    def go_above(self, x, y, z):
        """Planned move: MoveIt finds a collision-free path."""
        self.arm.set_pose_target(down_pose(x, y, z))
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return ok

    def straight_to_z(self, z):
        """Cartesian move: straight up or down to height z."""
        target = copy.deepcopy(self.arm.get_current_pose().pose)
        target.position.z = z
        try:
            plan, fraction = self.arm.compute_cartesian_path([target], 0.005)        # newer MoveIt
        except TypeError:
            plan, fraction = self.arm.compute_cartesian_path([target], 0.005, 0.0)   # older MoveIt
        if fraction < 0.95:
            rospy.logwarn('Straight move: only %.0f%% could be planned', fraction * 100)
            return False
        return self.arm.execute(plan, wait=True)

    def gripper_to(self, positions):
        ok = self.gripper.go(positions, wait=True)
        self.gripper.stop()
        return ok

    def _gz_request(self):
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = 'arm', 'palm'
        req.model_name_2, req.link_name_2 = 'red_cube', 'link'
        return req

    # ======================= The two worlds =======================
    def setup_scene(self):
        """Start clean: table and cube as obstacles in MoveIt."""
        self.scene.remove_attached_object(EE_LINK, name='cube')   # left over from an old run
        self.scene.remove_world_object('cube')
        rospy.sleep(0.5)
        self.add_box('table', 0.375, 0.0, 0.15, (0.25, 0.40, 0.30))
        self.add_box('cube', PICK_XY[0], PICK_XY[1], CUBE_Z, (CUBE_SIZE,) * 3)
        rospy.sleep(1.0)

    def hold_cube(self):
        """Fix the cube to the gripper in Gazebo AND in MoveIt."""
        ok = self.gz_attach.call(self._gz_request()).ok
        held = PoseStamped()
        held.header.frame_id = EE_LINK
        held.pose.position.z = GRASP_Z - CUBE_Z      # cube centre is 1 cm along ee_link's z (down)
        held.pose.orientation.w = 1.0
        self.scene.attach_box(EE_LINK, 'cube', held, (HELD_SIZE,) * 3, touch_links=TOUCH_LINKS)
        rospy.sleep(0.5)
        return ok

    def release_cube(self):
        """Let go in Gazebo AND in MoveIt."""
        ok = self.gz_detach.call(self._gz_request()).ok
        self.scene.remove_attached_object(EE_LINK, name='cube')
        rospy.sleep(0.5)
        self.scene.remove_world_object('cube')
        rospy.sleep(0.5)
        return ok

    # ======================= The task =======================
    def run(self):
        px, py = PICK_XY
        qx, qy = PLACE_XY
        self.setup_scene()

        self.check(self.go_named('home'), 'go home')
        self.check(self.gripper_to(GRIPPER_OPEN), 'open gripper')

        # ---- PICK ----
        self.check(self.go_above(px, py, ABOVE_Z), '1 pre-grasp: above the cube')
        self.scene.remove_world_object('cube')       # let the fingers go around it
        rospy.sleep(0.5)
        self.check(self.straight_to_z(GRASP_Z), '2 approach: straight down')
        self.check(self.gripper_to(GRIPPER_CUBE), '3 grasp: close the fingers')
        self.check(self.hold_cube(), '3 grasp: attach the cube')
        self.check(self.straight_to_z(ABOVE_Z), '4 lift: straight up')

        # ---- PLACE ----
        self.check(self.go_above(qx, qy, ABOVE_Z), '5 transport: above the place spot')
        self.check(self.straight_to_z(GRASP_Z + 0.005), '6 place: straight down')
        self.check(self.release_cube(), '7 release: detach the cube')
        self.check(self.gripper_to(GRIPPER_OPEN), '7 release: open the fingers')
        self.check(self.straight_to_z(ABOVE_Z), '8 retreat: straight up')
        self.add_box('cube', qx, qy, CUBE_Z, (CUBE_SIZE,) * 3)   # obstacle again, in its new place

        self.check(self.go_named('home'), 'go home')
        rospy.loginfo('Pick and place finished!')
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    PickPlace().run()
