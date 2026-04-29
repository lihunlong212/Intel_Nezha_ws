#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cargo_vision_node.py
====================
Visual-recognition node for the Intel Nezha competition drone.

Subscribes
----------
/camera/image_raw          sensor_msgs/Image   - raw camera feed
/cargo_vision/target_color std_msgs/String      - HSV colour range to track
                                                  format: "H_lo,H_hi,S_lo,S_hi,V_lo,V_hi"
                                                  default: red cargo

Publishes
---------
/cargo_vision/detection    cargo_handler/CargoDetection  - detection result per frame
/cargo_vision/image_debug  sensor_msgs/Image              - annotated debug image
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from cargo_handler.msg import CargoDetection


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _parse_hsv_range(text):
    """Parse "H_lo,H_hi,S_lo,S_hi,V_lo,V_hi" into two numpy arrays."""
    parts = [float(v.strip()) for v in text.split(",")]
    if len(parts) != 6:
        raise ValueError("Expected 6 comma-separated values, got %d" % len(parts))
    lower = np.array(parts[0:6:2], dtype=np.uint8)   # [H_lo, S_lo, V_lo]
    upper = np.array(parts[1:6:2], dtype=np.uint8)   # [H_hi, S_hi, V_hi]
    return lower, upper


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------

class CargoDetector:
    """Detect a single cargo item in an image by colour segmentation."""

    # Default: detect red cargo (red wraps around H=0/180 in OpenCV)
    DEFAULT_LOWER1 = np.array([0,   100, 100], dtype=np.uint8)
    DEFAULT_UPPER1 = np.array([10,  255, 255], dtype=np.uint8)
    DEFAULT_LOWER2 = np.array([170, 100, 100], dtype=np.uint8)
    DEFAULT_UPPER2 = np.array([180, 255, 255], dtype=np.uint8)

    # Minimum contour area to be considered a valid detection (pixels²)
    MIN_AREA = 500.0

    def __init__(self):
        self._lower1 = self.DEFAULT_LOWER1.copy()
        self._upper1 = self.DEFAULT_UPPER1.copy()
        self._lower2 = self.DEFAULT_LOWER2.copy()
        self._upper2 = self.DEFAULT_UPPER2.copy()
        self._use_dual_range = True   # needed for red hue wrap-around

    def set_colour_range(self, text):
        """Update tracking colour from "H_lo,H_hi,S_lo,S_hi,V_lo,V_hi" string."""
        try:
            lower, upper = _parse_hsv_range(text)
            self._lower1 = lower
            self._upper1 = upper
            self._use_dual_range = False
            rospy.loginfo("[cargo_vision] Colour range updated: lower=%s upper=%s",
                          lower, upper)
        except ValueError as exc:
            rospy.logwarn("[cargo_vision] Invalid colour range '%s': %s", text, exc)

    def detect(self, bgr_image):
        """
        Run colour-based detection on *bgr_image*.

        Returns
        -------
        detected : bool
        cx, cy   : float  – pixel centre (0 if not detected)
        area     : float  – contour area  (0 if not detected)
        contour  : ndarray or None
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        if self._use_dual_range:
            mask1 = cv2.inRange(hsv, self._lower1, self._upper1)
            mask2 = cv2.inRange(hsv, self._lower2, self._upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, self._lower1, self._upper1)

        # Morphological clean-up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, 0.0, None

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.MIN_AREA:
            return False, 0.0, 0.0, 0.0, None

        M = cv2.moments(best)
        if M["m00"] == 0:
            return False, 0.0, 0.0, 0.0, None

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        return True, cx, cy, area, best


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class CargoVisionNode:
    def __init__(self):
        rospy.init_node("cargo_vision_node", anonymous=False)

        self._bridge   = CvBridge()
        self._detector = CargoDetector()

        # Parameters
        self._cargo_id = rospy.get_param("~cargo_id", "cargo_1")
        self._publish_debug = rospy.get_param("~publish_debug_image", True)

        # Publishers
        self._det_pub = rospy.Publisher(
            "/cargo_vision/detection",
            CargoDetection,
            queue_size=10
        )
        self._debug_pub = rospy.Publisher(
            "/cargo_vision/image_debug",
            Image,
            queue_size=1
        )

        # Subscribers
        rospy.Subscriber("/camera/image_raw", Image,
                         self._image_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/cargo_vision/target_color", String,
                         self._colour_cb, queue_size=1)

        rospy.loginfo("[cargo_vision] Node started. Tracking cargo_id='%s'",
                      self._cargo_id)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _colour_cb(self, msg):
        self._detector.set_colour_range(msg.data)

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr("[cargo_vision] CvBridge error: %s", exc)
            return

        h, w = frame.shape[:2]
        detected, cx, cy, area, contour = self._detector.detect(frame)

        # Build and publish detection message
        det = CargoDetection()
        det.header.stamp    = msg.header.stamp
        det.header.frame_id = msg.header.frame_id
        det.detected   = detected
        det.cargo_id   = self._cargo_id
        det.center_x   = float(cx)
        det.center_y   = float(cy)
        det.area       = float(area)
        det.offset_x   = float(cx - w / 2.0) if detected else 0.0
        det.offset_y   = float(cy - h / 2.0) if detected else 0.0
        det.distance   = 0.0   # filled by depth sensor if available
        self._det_pub.publish(det)

        # Optional annotated debug image
        if self._publish_debug:
            debug = frame.copy()
            if detected and contour is not None:
                cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
                cv2.circle(debug, (int(cx), int(cy)), 8, (0, 0, 255), -1)
                cv2.putText(debug,
                            "cargo: ({:.0f},{:.0f}) area:{:.0f}".format(cx, cy, area),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
            else:
                cv2.putText(debug, "No cargo detected",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)
            try:
                self._debug_pub.publish(
                    self._bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
            except CvBridgeError as exc:
                rospy.logerr("[cargo_vision] Debug publish error: %s", exc)

    # ------------------------------------------------------------------

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    node = CargoVisionNode()
    node.spin()
