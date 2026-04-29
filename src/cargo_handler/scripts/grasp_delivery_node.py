#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grasp_delivery_node.py
======================
State-machine node for real-time cargo grasping and delivery.

Receives high-level commands via ROS, uses detection feedback from
cargo_vision_node to align the drone, then commands the gripper to
grasp or release cargo.

State machine
-------------
IDLE  ──(pickup cmd)──► SEARCHING ──(detected)──► ALIGNING
                                                        │
                                              (aligned & stable)
                                                        │
                                                    GRASPING ──(gripper closed)──► DELIVERING
                                                                                        │
                                                                                (at target)
                                                                                        │
                                                                                   RELEASING ──► IDLE

Subscribes
----------
/cargo_cmd                  cargo_handler/CargoCmd      - high-level command
/cargo_vision/detection     cargo_handler/CargoDetection - vision feedback
/drone/state                std_msgs/String              - flight-controller state (optional)

Publishes
---------
/cargo_handler/state        std_msgs/String              - current SM state
/drone/position_cmd         geometry_msgs/PoseStamped    - alignment position setpoint
/gripper_cmd                std_msgs/Bool                - True=close(grasp), False=open(release)
/cargo_handler/status       std_msgs/String              - human-readable status
"""

import math
import rospy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from cargo_handler.msg import CargoCmd, CargoDetection


# ---------------------------------------------------------------------------
# State definitions
# ---------------------------------------------------------------------------

class State:
    IDLE       = "IDLE"
    SEARCHING  = "SEARCHING"
    ALIGNING   = "ALIGNING"
    GRASPING   = "GRASPING"
    DELIVERING = "DELIVERING"
    RELEASING  = "RELEASING"


# ---------------------------------------------------------------------------
# Alignment controller (simple proportional)
# ---------------------------------------------------------------------------

class AlignController:
    """
    Convert pixel offsets from the camera centre into position increments.
    Uses proportional control; gain values are tunable via ROS params.
    """

    def __init__(self, kp_xy=0.001, kp_z=0.0, dead_zone_px=20.0):
        self.kp_xy      = kp_xy       # metres per pixel
        self.kp_z       = kp_z        # not used yet (depth sensor needed)
        self.dead_zone  = dead_zone_px

    def is_aligned(self, detection):
        """Return True when the cargo is centred within the dead zone."""
        return (abs(detection.offset_x) <= self.dead_zone and
                abs(detection.offset_y) <= self.dead_zone)

    def compute_correction(self, detection, current_pose):
        """
        Return a corrected PoseStamped that moves the drone toward alignment.
        Camera axes (x→right, y→down) map to drone body axes (x→forward assumed
        by camera-forward mount; adjust signs for your specific mounting).
        """
        pose = PoseStamped()
        pose.header.stamp    = rospy.Time.now()
        pose.header.frame_id = "world"

        # Copy current position
        pose.pose.position.x = current_pose.pose.position.x
        pose.pose.position.y = current_pose.pose.position.y
        pose.pose.position.z = current_pose.pose.position.z
        pose.pose.orientation = current_pose.pose.orientation

        # Proportional correction  (camera-down mount assumed)
        pose.pose.position.x -= self.kp_xy * detection.offset_y   # forward
        pose.pose.position.y -= self.kp_xy * detection.offset_x   # lateral

        return pose


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class GraspDeliveryNode:

    # How many consecutive aligned frames before we commit to grasping
    ALIGN_STABLE_FRAMES = 10

    # Seconds to wait for gripper to close/open
    GRIPPER_TIMEOUT = 2.0

    # Seconds to wait while searching before declaring failure
    SEARCH_TIMEOUT = 30.0

    def __init__(self):
        rospy.init_node("grasp_delivery_node", anonymous=False)

        # Parameters
        dead_zone = rospy.get_param("~alignment_dead_zone_px", 20.0)
        kp_xy     = rospy.get_param("~alignment_kp_xy", 0.001)
        self._align_ctrl = AlignController(kp_xy=kp_xy, dead_zone_px=dead_zone)

        # State
        self._state          = State.IDLE
        self._current_cmd    = None        # CargoCmd
        self._last_detection = None        # CargoDetection
        self._stable_count   = 0
        self._state_entry_t  = rospy.Time.now()

        # Current drone pose estimate (updated externally or initialised to zero)
        self._current_pose         = PoseStamped()
        self._current_pose.header.frame_id = "world"

        # Publishers
        self._state_pub   = rospy.Publisher("/cargo_handler/state",   String,       queue_size=1, latch=True)
        self._status_pub  = rospy.Publisher("/cargo_handler/status",  String,       queue_size=1, latch=True)
        self._pos_pub     = rospy.Publisher("/drone/position_cmd",    PoseStamped,  queue_size=1)
        self._gripper_pub = rospy.Publisher("/gripper_cmd",           Bool,         queue_size=1)

        # Subscribers
        rospy.Subscriber("/cargo_cmd",              CargoCmd,      self._cmd_cb,      queue_size=5)
        rospy.Subscriber("/cargo_vision/detection", CargoDetection, self._detection_cb, queue_size=10)
        rospy.Subscriber("/drone/pose",             PoseStamped,   self._pose_cb,     queue_size=10)

        # Control loop timer (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self._control_loop)

        self._publish_state(State.IDLE)
        rospy.loginfo("[grasp_delivery] Node started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg):
        rospy.loginfo("[grasp_delivery] Received command: action='%s' cargo_id='%s'",
                      msg.action, msg.cargo_id)
        if msg.action == CargoCmd.ACTION_ABORT:
            self._abort()
            return
        if msg.action in (CargoCmd.ACTION_PICKUP, CargoCmd.ACTION_DELIVER):
            if self._state != State.IDLE:
                rospy.logwarn("[grasp_delivery] Received new command while in state %s; ignoring.",
                              self._state)
                return
            self._current_cmd = msg
            if msg.action == CargoCmd.ACTION_PICKUP:
                self._transition(State.SEARCHING)
            else:
                # Deliver: go directly to DELIVERING with the target position
                self._transition(State.DELIVERING)
        else:
            rospy.logwarn("[grasp_delivery] Unknown action '%s'", msg.action)

    def _detection_cb(self, msg):
        self._last_detection = msg

    def _pose_cb(self, msg):
        self._current_pose = msg

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _control_loop(self, _event):
        state = self._state

        if state == State.IDLE:
            pass   # nothing to do

        elif state == State.SEARCHING:
            self._do_searching()

        elif state == State.ALIGNING:
            self._do_aligning()

        elif state == State.GRASPING:
            self._do_grasping()

        elif state == State.DELIVERING:
            self._do_delivering()

        elif state == State.RELEASING:
            self._do_releasing()

    # ---- SEARCHING -------------------------------------------------------

    def _do_searching(self):
        elapsed = (rospy.Time.now() - self._state_entry_t).to_sec()
        if elapsed > self.SEARCH_TIMEOUT:
            rospy.logwarn("[grasp_delivery] Search timeout. Returning to IDLE.")
            self._status("Search timeout – cargo not found.")
            self._transition(State.IDLE)
            return

        det = self._last_detection
        if det is not None and det.detected:
            rospy.loginfo("[grasp_delivery] Cargo detected. Switching to ALIGNING.")
            self._stable_count = 0
            self._transition(State.ALIGNING)

    # ---- ALIGNING --------------------------------------------------------

    def _do_aligning(self):
        det = self._last_detection
        if det is None or not det.detected:
            # Lost target – go back to searching
            rospy.logwarn("[grasp_delivery] Lost cargo during alignment. Back to SEARCHING.")
            self._stable_count = 0
            self._transition(State.SEARCHING)
            return

        if self._align_ctrl.is_aligned(det):
            self._stable_count += 1
            if self._stable_count >= self.ALIGN_STABLE_FRAMES:
                rospy.loginfo("[grasp_delivery] Alignment stable (%d frames). Grasping.",
                              self._stable_count)
                self._transition(State.GRASPING)
        else:
            self._stable_count = 0
            corrected = self._align_ctrl.compute_correction(det, self._current_pose)
            self._pos_pub.publish(corrected)
            self._status(
                "Aligning: offset=({:.1f},{:.1f}) px".format(det.offset_x, det.offset_y)
            )

    # ---- GRASPING --------------------------------------------------------

    def _do_grasping(self):
        elapsed = (rospy.Time.now() - self._state_entry_t).to_sec()
        if elapsed < 0.05:
            # First tick: command gripper to close
            rospy.loginfo("[grasp_delivery] Closing gripper.")
            self._gripper_pub.publish(Bool(data=True))
        elif elapsed >= self.GRIPPER_TIMEOUT:
            rospy.loginfo("[grasp_delivery] Gripper closed. Switching to DELIVERING.")
            self._transition(State.DELIVERING)

    # ---- DELIVERING ------------------------------------------------------

    def _do_delivering(self):
        if self._current_cmd is None:
            rospy.logwarn("[grasp_delivery] No active command in DELIVERING state.")
            self._transition(State.IDLE)
            return

        target = self._current_cmd.target_position
        pose = PoseStamped()
        pose.header.stamp    = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position   = target
        pose.pose.orientation = self._current_pose.pose.orientation
        self._pos_pub.publish(pose)

        # Check if we have reached the target
        if self._at_target(target):
            rospy.loginfo("[grasp_delivery] Reached delivery target. Releasing.")
            self._transition(State.RELEASING)

    def _at_target(self, target):
        """Return True when the drone is within 0.15 m of target in x-y plane."""
        dx = self._current_pose.pose.position.x - target.x
        dy = self._current_pose.pose.position.y - target.y
        dz = self._current_pose.pose.position.z - target.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist < 0.15

    # ---- RELEASING -------------------------------------------------------

    def _do_releasing(self):
        elapsed = (rospy.Time.now() - self._state_entry_t).to_sec()
        if elapsed < 0.05:
            rospy.loginfo("[grasp_delivery] Opening gripper.")
            self._gripper_pub.publish(Bool(data=False))
        elif elapsed >= self.GRIPPER_TIMEOUT:
            rospy.loginfo("[grasp_delivery] Cargo released. Back to IDLE.")
            self._status("Delivery complete.")
            self._current_cmd = None
            self._transition(State.IDLE)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state):
        rospy.loginfo("[grasp_delivery] %s → %s", self._state, new_state)
        self._state         = new_state
        self._state_entry_t = rospy.Time.now()
        self._publish_state(new_state)

    def _publish_state(self, state):
        self._state_pub.publish(String(data=state))

    def _status(self, text):
        self._status_pub.publish(String(data=text))

    def _abort(self):
        rospy.logwarn("[grasp_delivery] ABORT received. Opening gripper and returning to IDLE.")
        self._gripper_pub.publish(Bool(data=False))
        self._current_cmd = None
        self._transition(State.IDLE)

    # ------------------------------------------------------------------

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    node = GraspDeliveryNode()
    node.spin()
