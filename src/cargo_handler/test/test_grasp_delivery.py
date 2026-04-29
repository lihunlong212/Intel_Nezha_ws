#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_grasp_delivery.py
======================
Unit tests for the state-machine helpers in grasp_delivery_node.py.
Run with:  python3 -m pytest test/test_grasp_delivery.py -v
"""

import sys
import os
import math
import unittest
import types

# ---- stub ROS / message packages so the script can be imported standalone ----

rospy_stub = types.ModuleType("rospy")
rospy_stub.loginfo  = lambda *a, **k: None
rospy_stub.logwarn  = lambda *a, **k: None
rospy_stub.logerr   = lambda *a, **k: None
rospy_stub.get_param = lambda name, default=None: default


class _FakeTime:
    def __init__(self, secs=0.0):
        self.secs = secs
    def to_sec(self):
        return self.secs
    def __sub__(self, other):
        return _FakeTime(self.secs - other.secs)


rospy_stub.Time = types.SimpleNamespace(now=lambda: _FakeTime(0.0))
rospy_stub.Duration = lambda s: s
rospy_stub.Timer = lambda dur, cb: None


class _FakePublisher:
    def __init__(self, *a, **k):
        self.last_msg = None
    def publish(self, msg):
        self.last_msg = msg

rospy_stub.Publisher = lambda *a, **k: _FakePublisher()

class _FakeSubscriber:
    def __init__(self, *a, **k): pass
rospy_stub.Subscriber = _FakeSubscriber

rospy_stub.init_node = lambda *a, **k: None

sys.modules["rospy"] = rospy_stub

for mod in [
    "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "cargo_handler", "cargo_handler.msg",
]:
    sys.modules.setdefault(mod, types.ModuleType(mod))

class _FakeMsg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
    def __repr__(self):
        return "FakeMsg(%s)" % vars(self)

def _msg_factory(**kw):
    return _FakeMsg(**kw)

import functools

sys.modules["std_msgs.msg"].String = functools.partial(_msg_factory)
sys.modules["std_msgs.msg"].Bool   = functools.partial(_msg_factory)

class _FakePoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class _FakePoseStamped:
    class _Pose:
        class _Position:
            x = 0.0; y = 0.0; z = 0.0
        class _Orientation:
            x = 0.0; y = 0.0; z = 0.0; w = 1.0
        position    = _Position()
        orientation = _Orientation()
    class _Header:
        stamp    = None
        frame_id = ""
    pose   = _Pose()
    header = _Header()

sys.modules["geometry_msgs.msg"].Point        = _FakePoint
sys.modules["geometry_msgs.msg"].PoseStamped  = _FakePoseStamped

class _CargoCmd:
    ACTION_PICKUP  = "pickup"
    ACTION_DELIVER = "deliver"
    ACTION_ABORT   = "abort"

class _CargoDetection:
    detected = False
    cargo_id = ""
    center_x = 0.0; center_y = 0.0
    area     = 0.0
    offset_x = 0.0; offset_y = 0.0
    distance = 0.0

sys.modules["cargo_handler.msg"].CargoCmd       = _CargoCmd
sys.modules["cargo_handler.msg"].CargoDetection = _CargoDetection

# ---- Now import the module under test ------------------------------------

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from grasp_delivery_node import AlignController, State, GraspDeliveryNode


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestAlignController(unittest.TestCase):

    def setUp(self):
        self.ctrl = AlignController(kp_xy=0.001, dead_zone_px=20.0)

    def _det(self, ox, oy):
        d = _CargoDetection()
        d.offset_x = ox
        d.offset_y = oy
        return d

    def _pose(self, x=0.0, y=0.0, z=2.0):
        ps = _FakePoseStamped()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        return ps

    # --- is_aligned ---

    def test_aligned_within_dead_zone(self):
        self.assertTrue(self.ctrl.is_aligned(self._det(0, 0)))
        self.assertTrue(self.ctrl.is_aligned(self._det(19, -19)))
        self.assertTrue(self.ctrl.is_aligned(self._det(20, 20)))

    def test_not_aligned_outside_dead_zone(self):
        self.assertFalse(self.ctrl.is_aligned(self._det(21, 0)))
        self.assertFalse(self.ctrl.is_aligned(self._det(0, -21)))
        self.assertFalse(self.ctrl.is_aligned(self._det(100, 100)))

    # --- compute_correction ---

    def test_correction_moves_toward_target(self):
        pose = self._pose(x=0.0, y=0.0, z=2.0)
        # Cargo is 50 px to the right → drone should move right (positive y)
        det  = self._det(ox=50, oy=0)
        corrected = self.ctrl.compute_correction(det, pose)
        self.assertAlmostEqual(corrected.pose.position.x,  0.0, places=5)
        self.assertAlmostEqual(corrected.pose.position.y, -0.05, places=5)  # -kp_xy*ox
        self.assertAlmostEqual(corrected.pose.position.z,  2.0, places=5)

    def test_correction_forward(self):
        pose = self._pose(x=0.0, y=0.0, z=2.0)
        # Cargo is 50 px below centre → drone should move forward (x increases)
        det = self._det(ox=0, oy=50)
        corrected = self.ctrl.compute_correction(det, pose)
        self.assertAlmostEqual(corrected.pose.position.x, -0.05, places=5)
        self.assertAlmostEqual(corrected.pose.position.y,  0.0,  places=5)

    def test_altitude_preserved(self):
        pose = self._pose(x=1.0, y=2.0, z=3.5)
        det  = self._det(ox=100, oy=200)
        corrected = self.ctrl.compute_correction(det, pose)
        self.assertAlmostEqual(corrected.pose.position.z, 3.5, places=5)


class TestStateMachineTransitions(unittest.TestCase):
    """
    Test the state-machine logic without a live ROS environment
    by calling private methods directly.
    """

    def _make_node(self):
        node = GraspDeliveryNode.__new__(GraspDeliveryNode)
        # Minimal attribute initialisation (mirrors __init__)
        node._state          = State.IDLE
        node._current_cmd    = None
        node._last_detection = None
        node._stable_count   = 0
        node._state_entry_t  = _FakeTime(0.0)
        node._current_pose   = _FakePoseStamped()
        node._align_ctrl     = AlignController(kp_xy=0.001, dead_zone_px=20.0)
        node._state_pub      = _FakePublisher()
        node._status_pub     = _FakePublisher()
        node._pos_pub        = _FakePublisher()
        node._gripper_pub    = _FakePublisher()
        return node

    def _cmd(self, action, cargo_id="cargo_1", tx=0.0, ty=0.0, tz=0.0):
        cmd = _CargoCmd()
        cmd.action          = action
        cmd.cargo_id        = cargo_id
        cmd.target_position = _FakePoint(tx, ty, tz)
        return cmd

    def _detection(self, detected=True, ox=0.0, oy=0.0):
        d = _CargoDetection()
        d.detected = detected
        d.offset_x = ox
        d.offset_y = oy
        d.area     = 2000.0
        return d

    # ---- IDLE → SEARCHING via pickup command ----

    def test_pickup_cmd_transitions_to_searching(self):
        node = self._make_node()
        node._cmd_cb(self._cmd(action="pickup"))
        self.assertEqual(node._state, State.SEARCHING)

    # ---- IDLE → DELIVERING via deliver command ----

    def test_deliver_cmd_transitions_to_delivering(self):
        node = self._make_node()
        node._cmd_cb(self._cmd(action="deliver"))
        self.assertEqual(node._state, State.DELIVERING)

    # ---- Command while not IDLE is ignored ----

    def test_cmd_ignored_when_not_idle(self):
        node = self._make_node()
        node._state = State.ALIGNING
        node._cmd_cb(self._cmd(action="pickup"))
        self.assertEqual(node._state, State.ALIGNING)

    # ---- SEARCHING → ALIGNING on detection ----

    def test_searching_transitions_to_aligning_on_detection(self):
        node = self._make_node()
        node._state = State.SEARCHING
        node._last_detection = self._detection(detected=True)
        rospy_stub.Time.now = lambda: _FakeTime(1.0)
        node._state_entry_t  = _FakeTime(0.0)
        node._do_searching()
        self.assertEqual(node._state, State.ALIGNING)

    # ---- SEARCHING times out ----

    def test_searching_times_out(self):
        node = self._make_node()
        node._state         = State.SEARCHING
        node._state_entry_t = _FakeTime(0.0)
        rospy_stub.Time.now = lambda: _FakeTime(GraspDeliveryNode.SEARCH_TIMEOUT + 1)
        node._do_searching()
        self.assertEqual(node._state, State.IDLE)

    # ---- ALIGNING: stable frames lead to GRASPING ----

    def test_aligning_stable_leads_to_grasping(self):
        node = self._make_node()
        node._state          = State.ALIGNING
        node._last_detection = self._detection(detected=True, ox=0.0, oy=0.0)
        node._stable_count   = 0
        node._state_entry_t  = _FakeTime(0.0)
        for _ in range(GraspDeliveryNode.ALIGN_STABLE_FRAMES):
            node._do_aligning()
        self.assertEqual(node._state, State.GRASPING)

    # ---- ALIGNING: lost cargo → back to SEARCHING ----

    def test_aligning_lost_detection_returns_to_searching(self):
        node = self._make_node()
        node._state          = State.ALIGNING
        node._last_detection = self._detection(detected=False)
        node._do_aligning()
        self.assertEqual(node._state, State.SEARCHING)

    # ---- ABORT resets to IDLE ----

    def test_abort_from_any_state(self):
        for state in [State.SEARCHING, State.ALIGNING, State.GRASPING, State.DELIVERING]:
            node = self._make_node()
            node._state = state
            node._cmd_cb(self._cmd(action="abort"))
            self.assertEqual(node._state, State.IDLE,
                             msg="Expected IDLE after abort from %s" % state)

    # ---- _at_target ----

    def test_at_target_close_enough(self):
        node = self._make_node()
        node._current_pose.pose.position.x = 1.0
        node._current_pose.pose.position.y = 2.0
        node._current_pose.pose.position.z = 3.0
        target = _FakePoint(1.05, 2.05, 3.05)
        self.assertTrue(node._at_target(target))

    def test_at_target_too_far(self):
        node = self._make_node()
        node._current_pose.pose.position.x = 0.0
        node._current_pose.pose.position.y = 0.0
        node._current_pose.pose.position.z = 0.0
        target = _FakePoint(1.0, 1.0, 1.0)
        self.assertFalse(node._at_target(target))


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main()
