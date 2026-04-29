#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_cargo_vision.py
====================
Unit tests for CargoDetector (pure-Python, no ROS required).
Run with:  python3 -m pytest test/test_cargo_vision.py -v
"""

import sys
import os
import unittest
import numpy as np
import cv2

# Allow importing the script without ros dependencies
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

# Stub out rospy so the module can be imported without a ROS installation
import types
rospy_stub = types.ModuleType("rospy")
rospy_stub.loginfo  = lambda *a, **k: None
rospy_stub.logwarn  = lambda *a, **k: None
rospy_stub.logerr   = lambda *a, **k: None
rospy_stub.get_param = lambda name, default=None: default
sys.modules.setdefault("rospy", rospy_stub)

# Stub sensor_msgs, std_msgs, geometry_msgs, cv_bridge, cargo_handler
for mod in [
    "sensor_msgs", "sensor_msgs.msg",
    "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "cv_bridge",
    "cargo_handler", "cargo_handler.msg",
]:
    sys.modules.setdefault(mod, types.ModuleType(mod))

# Provide minimal stubs for the classes used at module level
sys.modules["cv_bridge"].CvBridge      = object
sys.modules["cv_bridge"].CvBridgeError = Exception

class _FakeMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

sys.modules["std_msgs.msg"].String   = _FakeMsg
sys.modules["sensor_msgs.msg"].Image = _FakeMsg

class _FakePoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

sys.modules["geometry_msgs.msg"].Point        = _FakePoint
sys.modules["cargo_handler.msg"].CargoDetection = _FakeMsg

from cargo_vision_node import CargoDetector, _parse_hsv_range


# ---------------------------------------------------------------------------
# Helpers to build synthetic images
# ---------------------------------------------------------------------------

def _make_red_image(size=(480, 640), rect=None):
    """Return a BGR image with a red rectangle drawn on a black background."""
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    if rect is None:
        h, w = size
        rect = (w // 4, h // 4, w // 2, h // 2)  # (x, y, width, height)
    x, y, rw, rh = rect
    img[y:y+rh, x:x+rw] = (0, 0, 200)   # BGR red
    return img


def _make_blue_image(size=(480, 640), rect=None):
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    if rect is None:
        h, w = size
        rect = (w // 4, h // 4, w // 2, h // 2)
    x, y, rw, rh = rect
    img[y:y+rh, x:x+rw] = (200, 0, 0)   # BGR blue
    return img


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestParseHsvRange(unittest.TestCase):

    def test_valid_string(self):
        lo, hi = _parse_hsv_range("0,10,100,255,100,255")
        np.testing.assert_array_equal(lo, [0, 100, 100])
        np.testing.assert_array_equal(hi, [10, 255, 255])

    def test_invalid_length_raises(self):
        with self.assertRaises(ValueError):
            _parse_hsv_range("0,10,100")

    def test_whitespace_tolerance(self):
        lo, hi = _parse_hsv_range(" 0 , 10 , 100 , 255 , 100 , 255 ")
        np.testing.assert_array_equal(lo, [0, 100, 100])
        np.testing.assert_array_equal(hi, [10, 255, 255])


class TestCargoDetectorDefault(unittest.TestCase):
    """Default detector looks for red cargo."""

    def setUp(self):
        self.det = CargoDetector()

    def test_detects_red_rectangle(self):
        img = _make_red_image()
        detected, cx, cy, area, contour = self.det.detect(img)
        self.assertTrue(detected)
        self.assertGreater(area, CargoDetector.MIN_AREA)
        self.assertIsNotNone(contour)

    def test_no_detection_on_black_image(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        detected, cx, cy, area, contour = self.det.detect(img)
        self.assertFalse(detected)
        self.assertEqual(cx, 0.0)
        self.assertEqual(cy, 0.0)
        self.assertIsNone(contour)

    def test_no_detection_on_blue_image(self):
        img = _make_blue_image()
        detected, *_ = self.det.detect(img)
        self.assertFalse(detected)

    def test_centre_accuracy(self):
        """Detected centre should be close to the actual rectangle centre."""
        h, w = 480, 640
        rw, rh = 200, 150
        rx = (w - rw) // 2
        ry = (h - rh) // 2
        img = _make_red_image(size=(h, w), rect=(rx, ry, rw, rh))
        detected, cx, cy, area, _ = self.det.detect(img)
        self.assertTrue(detected)
        expected_cx = rx + rw / 2.0
        expected_cy = ry + rh / 2.0
        self.assertAlmostEqual(cx, expected_cx, delta=10.0)
        self.assertAlmostEqual(cy, expected_cy, delta=10.0)

    def test_small_blob_ignored(self):
        """A blob smaller than MIN_AREA should not be reported as detected."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # 5×5 red square → area 25 < MIN_AREA=500
        img[100:105, 100:105] = (0, 0, 200)
        detected, *_ = self.det.detect(img)
        self.assertFalse(detected)


class TestCargoDetectorColourChange(unittest.TestCase):
    """Detector can be reconfigured to track blue cargo."""

    def setUp(self):
        self.det = CargoDetector()
        # Blue: H≈120 in OpenCV HSV (0-180 scale)
        self.det.set_colour_range("110,130,100,255,100,255")

    def test_detects_blue_after_reconfigure(self):
        img = _make_blue_image()
        detected, cx, cy, area, _ = self.det.detect(img)
        self.assertTrue(detected)
        self.assertGreater(area, CargoDetector.MIN_AREA)

    def test_no_longer_detects_red(self):
        img = _make_red_image()
        detected, *_ = self.det.detect(img)
        self.assertFalse(detected)

    def test_invalid_colour_string_keeps_previous(self):
        """An invalid colour string should not crash and keeps old settings."""
        self.det.set_colour_range("this,is,bad")   # should log a warning, not raise
        img = _make_blue_image()
        detected, *_ = self.det.detect(img)
        self.assertTrue(detected)


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main()
