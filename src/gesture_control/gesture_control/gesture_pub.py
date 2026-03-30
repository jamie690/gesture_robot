#!/usr/bin/env python3
import math
import time
import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class GesturePub(Node):
    """
    Intentional gesture interface for shared robot control.

    Publishes:
      /gesture     (std_msgs/String): "left", "right", "forward", "back",
                                      "fist", or "none"
      /pinch       (std_msgs/Float32): raw thumb-index distance
      /pinch_norm  (std_msgs/Float32): normalized pinch 0..1

    Interaction model:
      - System starts INACTIVE
      - Activate with two fingers up inside central box, palm facing camera,
        held stable for a short dwell
      - While ACTIVE, only gestures inside interaction zone are processed
      - Deactivate with two fingers up again anywhere in frame
      - Red border = inactive, Green border = active

    Direction mapping:
      hand left/right in image -> left/right
      hand up/down in image    -> forward/back
    """

    def __init__(self):
        super().__init__("gesture_pub")

        # ------------------------------------------------------------
        # ROS pubs
        # ------------------------------------------------------------
        self.gesture_pub = self.create_publisher(String, "/gesture", 10)
        self.pinch_pub = self.create_publisher(Float32, "/pinch", 10)
        self.pinch_norm_pub = self.create_publisher(Float32, "/pinch_norm", 10)
        self.image_pub = self.create_publisher(Image, "/gesture_camera/image_raw", 10)
        self.bridge = CvBridge()

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("flip", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("min_det_conf", 0.6)
        self.declare_parameter("min_track_conf", 0.6)

        self.declare_parameter("pinch_min", 0.02)
        self.declare_parameter("pinch_max", 0.18)

        # Interaction zone as fractions of image width/height
        self.declare_parameter("zone_x_min", 0.25)
        self.declare_parameter("zone_x_max", 0.75)
        self.declare_parameter("zone_y_min", 0.12)
        self.declare_parameter("zone_y_max", 0.72)

        # Neutral centre region inside active mode
        self.declare_parameter("neutral_deadzone_x", 0.10)
        self.declare_parameter("neutral_deadzone_y", 0.10)

        # Gesture filtering
        self.declare_parameter("gesture_hold_frames", 5)
        self.declare_parameter("activation_hold_frames", 10)
        self.declare_parameter("command_cooldown_s", 0.25)

        # Optional extra protection: reject low-confidence edge wandering
        self.declare_parameter("min_palm_facing_score", 0.35)

        # Window size
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 980)

        # Image publishing
        self.declare_parameter("publish_image", True)


        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.flip = bool(self.get_parameter("flip").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.min_det_conf = float(self.get_parameter("min_det_conf").value)
        self.min_track_conf = float(self.get_parameter("min_track_conf").value)
        self.pinch_min = float(self.get_parameter("pinch_min").value)
        self.pinch_max = float(self.get_parameter("pinch_max").value)

        self.zone_x_min = float(self.get_parameter("zone_x_min").value)
        self.zone_x_max = float(self.get_parameter("zone_x_max").value)
        self.zone_y_min = float(self.get_parameter("zone_y_min").value)
        self.zone_y_max = float(self.get_parameter("zone_y_max").value)

        self.neutral_deadzone_x = float(self.get_parameter("neutral_deadzone_x").value)
        self.neutral_deadzone_y = float(self.get_parameter("neutral_deadzone_y").value)

        self.gesture_hold_frames = int(self.get_parameter("gesture_hold_frames").value)
        self.activation_hold_frames = int(self.get_parameter("activation_hold_frames").value)
        self.command_cooldown_s = float(self.get_parameter("command_cooldown_s").value)
        self.min_palm_facing_score = float(self.get_parameter("min_palm_facing_score").value)

        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)

        self.publish_image = bool(self.get_parameter("publish_image").value)

        # ------------------------------------------------------------
        # Camera
        # ------------------------------------------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            raise RuntimeError("Could not open webcam")
        
        if self.show_window:
            cv2.namedWindow("Gesture Publisher (Intentional Control)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Gesture Publisher (Intentional Control)", self.window_width, self.window_height)

        # ------------------------------------------------------------
        # MediaPipe
        # ------------------------------------------------------------
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=self.min_det_conf,
            min_tracking_confidence=self.min_track_conf,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.active = False

        self._last_candidate = "none"
        self._same_count = 0
        self._stable_gesture = "none"

        self._last_activation_candidate = False
        self._activation_same_count = 0

        self._last_published_gesture = "none"
        self._last_publish_time = 0.0

        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info("gesture_pub started with activation mode.")

        self.toggle_latched = False
        self.mode_switch_cooldown_s = 0.75
        self.last_mode_switch_time = 0.0

    # ------------------------------------------------------------
    # Landmark helpers
    # ------------------------------------------------------------
    def _finger_extended(self, lm, tip_idx: int, pip_idx: int) -> bool:
        return lm[tip_idx].y < lm[pip_idx].y

    def _thumb_extended(self, lm) -> bool:
        return abs(lm[4].x - lm[2].x) > abs(lm[3].x - lm[2].x)

    def count_extended_fingers(self, lm):
        return {
            "thumb": self._thumb_extended(lm),
            "index": self._finger_extended(lm, 8, 6),
            "middle": self._finger_extended(lm, 12, 10),
            "ring": self._finger_extended(lm, 16, 14),
            "pinky": self._finger_extended(lm, 20, 18),
        }

    def palm_center(self, lm):
        x = (lm[0].x + lm[5].x + lm[9].x + lm[13].x + lm[17].x) / 5.0
        y = (lm[0].y + lm[5].y + lm[9].y + lm[13].y + lm[17].y) / 5.0
        return x, y

    def pinch_distance(self, lm) -> float:
        return math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y)

    def palm_facing_score(self, lm) -> float:
        """
        Cheap heuristic for 'roughly facing camera':
        if MCP spread is reasonably large relative to palm height,
        hand is less likely to be edge-on.
        """
        width = abs(lm[17].x - lm[5].x)
        height = abs(lm[0].y - lm[9].y) + 1e-6
        return width / height

    def hand_in_interaction_zone(self, palm_x: float, palm_y: float) -> bool:
        return (
            self.zone_x_min <= palm_x <= self.zone_x_max
            and self.zone_y_min <= palm_y <= self.zone_y_max
        )

    # ------------------------------------------------------------
    # Hand-shape classification
    # ------------------------------------------------------------
    def is_two_fingers_up(self, lm) -> bool:
        ext = self.count_extended_fingers(lm)
        return (
            ext["index"]
            and ext["middle"]
            and not ext["ring"]
            and not ext["pinky"]
        )

    def classify_shape(self, lm) -> str:
        ext = self.count_extended_fingers(lm)
        total_extended = sum(int(v) for v in ext.values())

        if total_extended >= 4:
            return "open"
        if total_extended <= 1:
            return "fist"
        return "none"

    def classify_direction(self, lm) -> str:
        palm_x, palm_y = self.palm_center(lm)

        dx = palm_x - 0.5
        dy = palm_y - 0.5

        # Neutral region
        if abs(dx) < self.neutral_deadzone_x and abs(dy) < self.neutral_deadzone_y:
            return "none"

        # Dominant axis wins
        if abs(dx) >= abs(dy):
            return "left" if dx < 0.0 else "right"
        else:
            # Image up = smaller y. User wants hand up/down -> robot forward/back.
            # Here:
            #   hand higher in image -> forward
            #   hand lower in image  -> back
            return "forward" if dy < 0.0 else "back"

    def classify_active_gesture(self, lm, in_zone: bool) -> str:
        """
        Only used while active.
        """
        if self.is_two_fingers_up(lm):
            return "none"  # reserve for mode toggle only

        shape = self.classify_shape(lm)

        if shape == "fist" and in_zone:
            return "fist"

        if shape == "open" and in_zone:
            return self.classify_direction(lm)

        return "none"

    # ------------------------------------------------------------
    # Filters
    # ------------------------------------------------------------
    def smooth_gesture(self, candidate: str) -> str:
        if candidate == self._last_candidate:
            self._same_count += 1
        else:
            self._last_candidate = candidate
            self._same_count = 1

        if self._same_count >= self.gesture_hold_frames:
            self._stable_gesture = self._last_candidate

        return self._stable_gesture

    def activation_dwell_ok(self, activation_candidate: bool) -> bool:
        if activation_candidate == self._last_activation_candidate:
            self._activation_same_count += 1
        else:
            self._last_activation_candidate = activation_candidate
            self._activation_same_count = 1

        return activation_candidate and self._activation_same_count >= self.activation_hold_frames

    def publish_with_cooldown(self, gesture: str):
        """
        Publishes stable gesture but limits rapid command flips.
        'none' is always allowed so the robot can stop receiving drive commands.
        """
        now = time.time()

        if gesture == "none":
            if gesture != self._last_published_gesture:
                self.gesture_pub.publish(String(data="none"))
                self._last_published_gesture = "none"
            return

        if gesture != self._last_published_gesture:
            if (now - self._last_publish_time) < self.command_cooldown_s:
                return
            self._last_publish_time = now

        self.gesture_pub.publish(String(data=gesture))
        self._last_published_gesture = gesture

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to read webcam frame.")
            return

        if self.flip:
            frame = cv2.flip(frame, 1)

        frame_h, frame_w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture_candidate = "none"
        stable_output = self._stable_gesture
        pinch = None
        pinch_norm = None

        palm_x = None
        palm_y = None
        in_zone = False
        facing_score = 0.0
        activation_candidate = False

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            lm = hand_landmarks.landmark

            self.mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_styles.get_default_hand_landmarks_style(),
                self.mp_styles.get_default_hand_connections_style(),
            )

            pinch = self.pinch_distance(lm)
            pinch_norm = clamp(
                (pinch - self.pinch_min) / (self.pinch_max - self.pinch_min),
                0.0,
                1.0,
            )

            palm_x, palm_y = self.palm_center(lm)
            in_zone = self.hand_in_interaction_zone(palm_x, palm_y)
            facing_score = self.palm_facing_score(lm)

            two_up = self.is_two_fingers_up(lm)
            palm_ok = facing_score >= self.min_palm_facing_score
            now = time.time()

            if two_up and palm_ok:
                if not self.toggle_latched:
                    if not self.active:
                        if in_zone and self.activation_dwell_ok(True):
                            if (now - self.last_mode_switch_time) > self.mode_switch_cooldown_s:
                                self.active = True
                                self.toggle_latched = True
                                self.last_mode_switch_time = now
                                self._stable_gesture = "none"
                                self._last_candidate = "none"
                                self._same_count = 0
                                self.get_logger().info("Gesture control ACTIVATED")
                    else:
                        if self.activation_dwell_ok(True):
                            if (now - self.last_mode_switch_time) > self.mode_switch_cooldown_s:
                                self.active = False
                                self.toggle_latched = True
                                self.last_mode_switch_time = now
                                self._stable_gesture = "none"
                                self._last_candidate = "none"
                                self._same_count = 0
                                self.publish_with_cooldown("none")
                                self.get_logger().info("Gesture control DEACTIVATED")
                else:
                    self.activation_dwell_ok(False)
            else:
                self.toggle_latched = False
                self.activation_dwell_ok(False)

            if self.active:
                gesture_candidate = self.classify_active_gesture(lm, in_zone)
                stable_output = self.smooth_gesture(gesture_candidate)
            else:
                stable_output = self.smooth_gesture("none")

        else:
            stable_output = self.smooth_gesture("none")
            self.activation_dwell_ok(False)
            self.toggle_latched = False
        

        # Publish pinch topics
        if pinch is not None:
            self.pinch_pub.publish(Float32(data=float(pinch)))
            self.pinch_norm_pub.publish(Float32(data=float(pinch_norm)))
        else:
            self.pinch_pub.publish(Float32(data=0.0))
            self.pinch_norm_pub.publish(Float32(data=1.0))

        # Publish gesture
        if self.active:
            self.publish_with_cooldown(stable_output)
        else:
            self.publish_with_cooldown("none")

        # --------------------------------------------------------
        # UI overlays
        # --------------------------------------------------------
        zone_px = (
            int(self.zone_x_min * frame_w),
            int(self.zone_y_min * frame_h),
            int(self.zone_x_max * frame_w),
            int(self.zone_y_max * frame_h),
        )

        zx0, zy0, zx1, zy1 = zone_px
        cv2.rectangle(frame, (zx0, zy0), (zx1, zy1), (255, 255, 0), 2)

        # Neutral centre box
        cx = frame_w // 2
        cy = frame_h // 2
        ndx = int(self.neutral_deadzone_x * frame_w)
        ndy = int(self.neutral_deadzone_y * frame_h)
        cv2.rectangle(frame, (cx - ndx, cy - ndy), (cx + ndx, cy + ndy), (120, 120, 120), 1)
        cv2.line(frame, (cx, 0), (cx, frame_h), (80, 80, 80), 1)
        cv2.line(frame, (0, cy), (frame_w, cy), (80, 80, 80), 1)

        # Palm marker
        if palm_x is not None and palm_y is not None:
            px = int(palm_x * frame_w)
            py = int(palm_y * frame_h)
            color = (0, 255, 255) if in_zone else (0, 140, 255)
            cv2.circle(frame, (px, py), 8, color, -1)

        # Border
        border_color = (0, 255, 0) if self.active else (0, 0, 255)
        cv2.rectangle(frame, (3, 3), (frame_w - 4, frame_h - 4), border_color, 6)

        # Text
        mode_text = "ACTIVE" if self.active else "INACTIVE"
        cv2.putText(
            frame,
            f"Mode: {mode_text}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            border_color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Candidate: {gesture_candidate}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Stable: {stable_output if self.active else 'none'}",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"In zone: {in_zone}",
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Palm facing score: {facing_score:.2f}",
            (20, 158),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            "2 fingers in box = activate | 2 fingers anywhere = deactivate",
            (20, frame_h - 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "Open hand = drive | Fist = pick | Centre = neutral",
            (20, frame_h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        
        # Publish image
        if self.publish_image:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish webcam image: {e}")

        if self.show_window:
            cdisplay_frame = cv2.resize(
                frame,
                (self.window_width, self.window_height),
                interpolation=cv2.INTER_LINEAR,
            )
            cv2.imshow("Gesture Publisher (Intentional Control)", cdisplay_frame)

            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                self.get_logger().info("Quit key pressed, shutting down gesture_pub.")
                rclpy.shutdown()

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GesturePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()