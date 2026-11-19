#!/usr/bin/env python3
# cube_finder.py - find the red, green and blue cubes in the camera image
#                  and publish where each one is on the table (Module 3.6.2, Step 6)

import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

# ---------------- Colours (OpenCV HSV: H 0-179, S 0-255, V 0-255) ----------------
# Red sits at both ends of the hue circle, so it needs two ranges.
COLOURS = {
    'red':   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 120, 70), (80, 255, 255))],
    'blue':  [((100, 120, 70), (130, 255, 255))],
}
MIN_AREA = 100            # ignore specks smaller than this (pixels)

# The arm is spawned in /arm (Module 3.6.2), so Gazebo puts the camera topics in /arm too.
# Check with:  rostopic list | grep image_raw
CAMERA = '/arm/camera'

# ---------------- The table ----------------
TABLE_TOP_Z = 0.30        # m, world frame
CUBE_SIZE = 0.04          # the camera sees the top face, 0.34 m above the floor


class CubeFinder:
    def __init__(self):
        rospy.init_node('cube_finder')
        self.bridge = CvBridge()

        # The lens: focal lengths and image centre, in pixels
        info = rospy.wait_for_message(CAMERA + '/camera_info', CameraInfo)
        self.fx, self.fy = info.K[0], info.K[4]
        self.cx, self.cy = info.K[2], info.K[5]
        rospy.loginfo('Camera: fx %.1f, fy %.1f, centre (%.1f, %.1f)', self.fx, self.fy, self.cx, self.cy)

        # TF tells us where the camera is in the world
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pubs = {c: rospy.Publisher('/cube/' + c, PointStamped, queue_size=1) for c in COLOURS}
        self.debug_pub = rospy.Publisher('/cube_finder/detections', Image, queue_size=1)
        rospy.Subscriber(CAMERA + '/image_raw', Image, self.on_image, queue_size=1, buff_size=2**24)

    def find_colour(self, hsv, ranges):
        """Return the biggest blob of this colour as a contour, or None."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        biggest = max(contours, key=cv2.contourArea)
        return biggest if cv2.contourArea(biggest) >= MIN_AREA else None

    def pixel_to_world(self, u, v):
        """Turn a pixel (u, v) into a point on the table, in the world frame."""
        try:
            cam = self.tf_buffer.lookup_transform('world', 'camera_optical_link',
                                                  rospy.Time(0), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5, 'No TF from world to camera_optical_link yet')
            return None

        # The camera looks straight down, so the distance along its z axis
        # is simply its height above the top face of the cube.
        depth = cam.transform.translation.z - (TABLE_TOP_Z + CUBE_SIZE)

        # Pinhole camera: pixel offset from the centre x depth / focal length
        p = PointStamped()
        p.header.frame_id = 'camera_optical_link'
        p.point.x = (u - self.cx) * depth / self.fx
        p.point.y = (v - self.cy) * depth / self.fy
        p.point.z = depth

        world = tf2_geometry_msgs.do_transform_point(p, cam)
        world.point.z = TABLE_TOP_Z + CUBE_SIZE / 2      # report the cube's centre
        return world

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for colour, ranges in COLOURS.items():
            blob = self.find_colour(hsv, ranges)
            if blob is None:
                continue
            m = cv2.moments(blob)
            u, v = m['m10'] / m['m00'], m['m01'] / m['m00']     # centre of the blob
            point = self.pixel_to_world(u, v)
            if point is None:
                continue
            point.header.stamp = msg.header.stamp
            self.pubs[colour].publish(point)

            # draw what we found, for /cube_finder/detections
            cv2.drawContours(image, [blob], -1, (255, 255, 255), 2)
            cv2.putText(image, '%s (%.3f, %.3f)' % (colour, point.point.x, point.point.y),
                        (int(u) + 15, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))


if __name__ == '__main__':
    CubeFinder()
    rospy.spin()
