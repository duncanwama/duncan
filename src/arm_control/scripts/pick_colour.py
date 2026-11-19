#!/usr/bin/env python3
# pick_colour.py - ask the camera where the cube of one colour is, pick it,
#                  and place it on the white drop zone (Module 3.6.2, Step 7)
# Usage:  rosrun arm_control pick_colour.py green

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

# ---------------- Names ----------------
NS = '/arm'
DESC = '/arm/robot_description'
ARM_GROUP = 'arm'
GRIPPER_GROUP = 'gripper'            # the name you chose in the Setup Assistant
EE_LINK = 'ee_link'
TOUCH_LINKS = ['palm', 'left_finger', 'right_finger']
COLOURS = ['red', 'green', 'blue']   # NEW: Gazebo models are red_cube, green_cube, blue_cube

# ---------------- Positions (m, world frame) ----------------
PLACE_XY = (0.30, 0.00)              # NEW: the white drop zone
ABOVE_Z = 0.38
GRASP_Z = 0.33
CUBE_Z = 0.32
CUBE_SIZE = 0.04
HELD_SIZE = 0.036

# ---------------- Gripper (m per finger) ----------------
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_CUBE = [0.019, 0.019]


def down_pose(x, y, z):
    """ee_link at (x, y, z), fingers pointing straight down."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y = 0.0, 1.0
    pose.orientation.z, pose.orientation.w = 0.0, 0.0
    return pose


class PickColour:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_colour')

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
        rospy.sleep(1.0)

    # ======================= Helpers (as in pick_place.py) =======================
    def check(self, ok, stage):
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
        self.arm.set_pose_target(down_pose(x, y, z))
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return ok

    def straight_to_z(self, z):
        target = copy.deepcopy(self.arm.get_current_pose().pose)
        target.position.z = z
        try:
            plan, fraction = self.arm.compute_cartesian_path([target], 0.005)
        except TypeError:
            plan, fraction = self.arm.compute_cartesian_path([target], 0.005, 0.0)
        if fraction < 0.95:
            rospy.logwarn('Straight move: only %.0f%% could be planned', fraction * 100)
            return False
        return self.arm.execute(plan, wait=True)

    def gripper_to(self, positions):
        ok = self.gripper.go(positions, wait=True)
        self.gripper.stop()
        return ok

    def _gz_request(self, colour):
        req = AttachRequest()
        req.model_name_1, req.link_name_1 = 'arm', 'palm'
        req.model_name_2, req.link_name_2 = colour + '_cube', 'link'   # NEW: any colour
        return req

    # ======================= NEW: ask the camera =======================
    def find_cubes(self):
        """Wait for cube_finder's answer for each colour. Returns {colour: (x, y)}."""
        found = {}
        for colour in COLOURS:
            try:
                msg = rospy.wait_for_message('/cube/' + colour, PointStamped, timeout=3.0)
            except rospy.ROSException:
                rospy.logwarn('The camera cannot see a %s cube', colour)
                continue
            found[colour] = (msg.point.x, msg.point.y)
            rospy.loginfo('Camera: %s cube at (%.3f, %.3f)', colour, msg.point.x, msg.point.y)
        return found

    # ======================= The two worlds =======================
    def setup_scene(self, found):
        """NEW: the table, plus every cube the camera found, as obstacles in MoveIt."""
        for colour in COLOURS:
            self.scene.remove_attached_object(EE_LINK, name='cube_' + colour)
            self.scene.remove_world_object('cube_' + colour)
        rospy.sleep(0.5)
        self.add_box('table', 0.375, 0.0, 0.15, (0.25, 0.40, 0.30))
        for colour, (x, y) in found.items():
            self.add_box('cube_' + colour, x, y, CUBE_Z, (CUBE_SIZE,) * 3)
        rospy.sleep(1.0)

    def hold_cube(self, colour):
        ok = self.gz_attach.call(self._gz_request(colour)).ok
        held = PoseStamped()
        held.header.frame_id = EE_LINK
        held.pose.position.z = GRASP_Z - CUBE_Z
        held.pose.orientation.w = 1.0
        self.scene.attach_box(EE_LINK, 'cube_' + colour, held, (HELD_SIZE,) * 3,
                              touch_links=TOUCH_LINKS)
        rospy.sleep(0.5)
        return ok

    def release_cube(self, colour):
        ok = self.gz_detach.call(self._gz_request(colour)).ok
        self.scene.remove_attached_object(EE_LINK, name='cube_' + colour)
        rospy.sleep(0.5)
        self.scene.remove_world_object('cube_' + colour)
        rospy.sleep(0.5)
        return ok

    # ======================= The task =======================
    def run(self, colour):
        # NEW: get the arm out of the picture, then look
        self.check(self.go_named('home'), 'go home, out of the camera view')
        self.check(self.gripper_to(GRIPPER_OPEN), 'open gripper')
        rospy.sleep(1.0)                                  # let a fresh image arrive
        found = self.find_cubes()
        self.check(colour in found, 'camera found the %s cube' % colour)
        px, py = found[colour]
        qx, qy = PLACE_XY
        self.setup_scene(found)

        # ---- PICK, at the position the camera reported ----
        self.check(self.go_above(px, py, ABOVE_Z), '1 pre-grasp: above the %s cube' % colour)
        self.scene.remove_world_object('cube_' + colour)
        rospy.sleep(0.5)
        self.check(self.straight_to_z(GRASP_Z), '2 approach: straight down')
        self.check(self.gripper_to(GRIPPER_CUBE), '3 grasp: close the fingers')
        self.check(self.hold_cube(colour), '3 grasp: attach the cube')
        self.check(self.straight_to_z(ABOVE_Z), '4 lift: straight up')

        # ---- PLACE on the drop zone ----
        self.check(self.go_above(qx, qy, ABOVE_Z), '5 transport: above the drop zone')
        self.check(self.straight_to_z(GRASP_Z + 0.005), '6 place: straight down')
        self.check(self.release_cube(colour), '7 release: detach the cube')
        self.check(self.gripper_to(GRIPPER_OPEN), '7 release: open the fingers')
        self.check(self.straight_to_z(ABOVE_Z), '8 retreat: straight up')
        self.add_box('cube_' + colour, qx, qy, CUBE_Z, (CUBE_SIZE,) * 3)

        self.check(self.go_named('home'), 'go home')
        rospy.loginfo('The %s cube is on the drop zone!', colour)
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)       # NEW: the colour from the command line
    if len(args) != 2 or args[1] not in COLOURS:
        print('Usage: rosrun arm_control pick_colour.py red|green|blue')
        sys.exit(1)
    PickColour().run(args[1])
