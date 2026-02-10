#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# =========================
# Joy mapping (as requested)
# =========================
AXIS_SWAY = 0   # left/right (optional - not used in mixing below)
AXIS_FWD  = 1   # w=-1, s=+1
AXIS_YAW  = 2   # a=-1, d=+1
AXIS_HEAVE = 3

BTN_Z_HOLD_ON = 3      # depth-hold button -> here we do Z-hold (hold current Z position)
BTN_Z_HOLD_OFF = 0

BTN_CAM_UP = 12     # tilt up
BTN_CAM_DN = 13    # tilt down

BTN_LIGHT_DN = 14     # brightness down
BTN_LIGHT_UP = 15     # brightness up

BTN_SCALE_UP = 1    # output scale up
BTN_SCALE_DN = 2    # output scale down

BTN_ARM_OFF = 8
BTN_ARM_ON = 9

DEADZONE = 0.08
RATE_HZ = 50.0
TIMEOUT_SEC = 0.3

# =========================
# Command scaling
# =========================
STEP_XY = 5.0
STEP_Z  = 5.0
MAX_CMD = 30.0

CAM_STEP = 0.03
CAM_MIN = -0.785398
CAM_MAX = 0.785398

LIGHT_MIN = 0
LIGHT_MAX = 3

THROTTLE_LEVELS = [0.25, 0.50, 0.75, 1.00]
THROTTLE_DEFAULT_INDEX = 1      # 0.50

# =========================
# Z-hold control (PI recommended)
# =========================
Z_KP = 2.0
Z_KI = 0.4          # set 0.0 to disable integral
I_CLAMP = 10.0      # integral anti-windup clamp in "cmd units"
LOCK_ON_RELEASE = True  # if True: target locks when heave buttons are released

# =========================
# IMPORTANT: Heave sign convention
# =========================
# We define "H > 0" as "try to increase Z (go upward in z-up world)".
# Depending on your model/plugin, sending positive cmd_thrust may move up or down.
# Use this to flip the heave command if Z-hold diverges.
HEAVE_SIGN = +1.0   # try +1.0 first; if it runs away, set to -1.0

# =========================
# Helpers
# =========================
def dz(x: float, deadzone: float) -> float:
    return 0.0 if abs(x) < deadzone else x

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

class JoyToThrusters(Node):
    def __init__(self):
        super().__init__("joy_to_thrusters_zh")

        self.sub_joy = self.create_subscription(Joy, "/joy", self.cb_joy, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/model/bluerov2/odometry", self.cb_odom, 10
        )

        self.pub = {}
        for i in range(1, 7):
            topic = f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust"
            self.pub[i] = self.create_publisher(Float64, topic, 10)

        self.last_axes = []
        self.last_buttons = []
        self.last_joy_time = None

        # ARM (control enable)
        self.armed = False      # start DISARMED
        self._last_arm_on = 0
        self._last_arm_off = 0

        # Z state (world z-up)
        self.z_now = None

        # Z-hold state
        self.z_hold = False
        self.z_target = None
        self._last_hold_btn = 0
        self._last_hold_off_btn = 0

        # OUTPUT SCALE
        self.throttle_index = THROTTLE_DEFAULT_INDEX
        self.throttle_scale = THROTTLE_LEVELS[self.throttle_index]
        self._last_scale_up = 0
        self._last_scale_dn = 0

        # LIGHT LEVEL 0..3
        self.light_level = 0
        self._last_light_up = 0
        self._last_light_dn = 0

        # CAMERA TILT
        self.cam_tilt = 0.0
        self.pub_cam_tilt = self.create_publisher(
            Float64,
            "/model/bluerov2/joint/camera_tilt_joint/cmd_pos",
            10
        )

        # manual heave tracking for lock-on-release behavior
        self._heave_was_active = False

        # integral state
        self.i_term = 0.0

        self.last_cmd = [0.0] * 6
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)


        self.get_logger().info(
            "joy_to_thrusters_zh started\n"
            f"- rate={RATE_HZ}Hz, timeout={TIMEOUT_SEC}s\n"
            f"- axes: FWD=axes[{AXIS_FWD}], YAW=axes[{AXIS_YAW}]\n"
            f"- STEP_XY={STEP_XY}, STEP_Z={STEP_Z}, MAX_CMD={MAX_CMD}\n"
            f"- Z_KP={Z_KP}, Z_KI={Z_KI}, I_CLAMP={I_CLAMP}, LOCK_ON_RELEASE={LOCK_ON_RELEASE}\n"
            f"- HEAVE_SIGN={HEAVE_SIGN}  (H>0 means 'increase Z')"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_joy(self, msg: Joy):
        self.last_axes = list(msg.axes)
        self.last_buttons = list(msg.buttons)
        self.last_joy_time = self.get_clock().now()

    def cb_odom(self, msg: Odometry):
        self.z_now = float(msg.pose.pose.position.z)

    # -------------------------
    # Input helpers
    # -------------------------
    def _get_axis(self, idx: int) -> float:
        return float(self.last_axes[idx]) if idx < len(self.last_axes) else 0.0

    def _get_btn(self, idx: int) -> int:
        return int(self.last_buttons[idx]) if idx < len(self.last_buttons) else 0

    # -------------------------
    # Output helpers
    # -------------------------
    def publish_cmd(self, cmd6):
        for i in range(1, 7):
            m = Float64()
            m.data = float(cmd6[i - 1])
            self.pub[i].publish(m)

    def mix_like_keyboard(self, F: float, Y: float, H: float):
        """
        Matches your keyboard-style mixing:
          t1 = -F - Y (+bias)
          t2 = -F + Y
          t3 =  F + Y (+bias)
          t4 =  F - Y
          t5,t6 = heave
        """
        bias = 2.0 if abs(F) > 1e-6 else 0.0
        bias *= 1.0 if F > 0 else -1.0

        t1 = -F - Y + bias
        t2 = -F + Y
        t3 =  F + Y + bias
        t4 =  F - Y

        # Heave mapping (keep your original sign here: t5=t6=-H)
        # We'll apply HEAVE_SIGN earlier so H means "increase Z" in our control law.
        t5 = -H
        t6 = -H

        return [clamp(v, -MAX_CMD, MAX_CMD) for v in (t1, t2, t3, t4, t5, t6)]

    # -------------------------
    # Main loop
    # -------------------------
    def tick(self):
        # No joystick yet -> all zero
        if self.last_joy_time is None:
            self._stop()
            return

        # Timeout -> all zero
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > TIMEOUT_SEC:
            self._stop()
            return

        dt = 1.0 / RATE_HZ

        # --------- ARM ON/OFF ---------
        arm_on = self._get_btn(BTN_ARM_ON)
        arm_off = self._get_btn(BTN_ARM_OFF)

        if arm_on == 1 and self._last_arm_on == 0:
            self.armed = True
            self.get_logger().info("CONTROL: ARMED")

        if arm_off == 1 and self._last_arm_off == 0:
            self.armed = False
            self.z_hold = False
            self.z_target = None
            self.i_term = 0.0
            self._heave_was_active = False
            self._stop()
            self.get_logger().info("CONTROL: DISARMED")

        self._last_arm_on = arm_on
        self._last_arm_off = arm_off

        if not self.armed:
            self._stop()
            return

        # --------- Z-hold ON ---------
        hold_on_btn = self._get_btn(BTN_Z_HOLD_ON)
        if hold_on_btn == 1 and self._last_hold_btn == 0:
            if not self.z_hold:
                self.z_hold = True
                self.z_target = None
                self.i_term = 0.0
                self._heave_was_active = False
                self.get_logger().info("Z hold ON (target will lock).")

        self._last_hold_btn = hold_on_btn

        # --------- Z-hold ON ---------
        hold_off_btn = self._get_btn(BTN_Z_HOLD_OFF)
        if hold_off_btn == 1 and self._last_hold_off_btn == 0:
            if self.z_hold:
                self.z_hold = False
                self.z_target = None
                self.i_term = 0.0
                self._heave_was_active = False
                self.get_logger().info("Z hold OFF")
        self._last_hold_off_btn = hold_off_btn

        # --------- output scale (single click) ---------
        s_up = self._get_btn(BTN_SCALE_UP)
        s_dn = self._get_btn(BTN_SCALE_DN)

        if s_up == 1 and self._last_scale_up == 0:
            if self.throttle_index < len(THROTTLE_LEVELS) - 1:
                self.throttle_index += 1
                self.throttle_scale = THROTTLE_LEVELS[self.throttle_index]
                self.get_logger().info(
                    f"throttle_scale = {self.throttle_scale:.2f} ({int(self.throttle_scale*100)}%)"
                )

        if s_dn == 1 and self._last_scale_dn == 0:
            if self.throttle_index > 0:
                self.throttle_index -= 1
                self.throttle_scale = THROTTLE_LEVELS[self.throttle_index]
                self.get_logger().info(
                    f"throttle_scale = {self.throttle_scale:.2f} ({int(self.throttle_scale*100)}%)"
                )

        self._last_scale_up = s_up
        self._last_scale_dn = s_dn

        # --------- light level (single click) ---------
        l_up = self._get_btn(BTN_LIGHT_UP)
        l_dn = self._get_btn(BTN_LIGHT_DN)

        if l_up == 1 and self._last_light_up == 0:
            self.light_level = min(self.light_level + 1, LIGHT_MAX)
            self.get_logger().info(f"light_level = {self.light_level}")

        if l_dn == 1 and self._last_light_dn == 0:
            self.light_level = max(self.light_level - 1, LIGHT_MIN)
            self.get_logger().info(f"light_level = {self.light_level}")

        self._last_light_up = l_up
        self._last_light_dn = l_dn

        # --------- camera tilt (hold) ---------
        cu = self._get_btn(BTN_CAM_UP)
        cd = self._get_btn(BTN_CAM_DN)

        prev = self.cam_tilt
        if cu:
            self.cam_tilt = clamp(self.cam_tilt - CAM_STEP, CAM_MIN, CAM_MAX)
        if cd:
            self.cam_tilt = clamp(self.cam_tilt + CAM_STEP, CAM_MIN, CAM_MAX)

        if self.cam_tilt != prev:
            m = Float64()
            m.data = float(self.cam_tilt)
            self.pub_cam_tilt.publish(m)
            # self.get_logger().info(f"cam_tilt = {self.cam_tilt:.3f} rad")

        # --------- read planar inputs ---------
        axis_fwd = dz(self._get_axis(AXIS_FWD), DEADZONE)
        F = (-axis_fwd) * STEP_XY

        axis_yaw = dz(self._get_axis(AXIS_YAW), DEADZONE)
        Y = (-axis_yaw) * STEP_XY

        # --------- manual heave buttons ---------
        axis_heave = dz(self._get_axis(AXIS_HEAVE), DEADZONE)
        H_manual = (-axis_heave) * STEP_Z

        heave_active = abs(H_manual) > 1e-6

        # --------- Z hold control ---------
        if self.z_hold and (self.z_now is not None):
            # Establish/Update target
            if self.z_target is None:
                self.z_target = float(self.z_now)
                self.i_term = 0.0
                self.get_logger().info(f"Z locked at {self.z_target:.3f}")

            # If user is actively commanding heave, decide how to behave
            if heave_active:
                if LOCK_ON_RELEASE:
                    # While user holds buttons, pass manual heave through and
                    # keep moving the target along with current Z (so release locks new position)
                    self.z_target = float(self.z_now)
                    self.i_term = 0.0
                    self._heave_was_active = True
                    H = H_manual
                else:
                    # Another mode: treat buttons as "nudge target" rather than raw thrust.
                    # This keeps hold active while changing setpoint.
                    # Positive H_manual -> increase Z_target.
                    self.z_target += (H_manual / max(STEP_Z, 1e-6)) * 0.02  # 2cm per tick per full press
                    self.z_target = float(self.z_target)
                    self._heave_was_active = True
                    # continue to compute hold output below
                    H = None
            else:
                # If we just released heave, lock at current Z once
                if self._heave_was_active and LOCK_ON_RELEASE:
                    self.z_target = float(self.z_now)
                    self.i_term = 0.0
                    self.get_logger().info(f"Z re-locked at {self.z_target:.3f}")
                self._heave_was_active = False
                H = None

            # If not using raw manual pass-through, compute PI hold output
            if H is None:
                err = (self.z_target - self.z_now)  # +err means "need to increase Z"
                p = Z_KP * err * STEP_Z

                if Z_KI != 0.0:
                    self.i_term += (Z_KI * err * STEP_Z) * dt
                    self.i_term = clamp(self.i_term, -I_CLAMP, I_CLAMP)
                else:
                    self.i_term = 0.0

                H_hold = p + self.i_term

                # Convert to our chosen convention: H>0 should increase Z.
                # If model sign is opposite, flip HEAVE_SIGN.
                H = HEAVE_SIGN * H_hold

                H = clamp(H, -MAX_CMD, MAX_CMD)

            else:
                # Manual pass-through: still apply HEAVE_SIGN so "ASCEND" means increase Z
                H = HEAVE_SIGN * H
                H = clamp(H, -MAX_CMD, MAX_CMD)

        else:
            # no hold -> pure manual heave
            self.z_target = None
            self.i_term = 0.0
            self._heave_was_active = False

            H = HEAVE_SIGN * H_manual
            H = clamp(H, -MAX_CMD, MAX_CMD)

        F *= self.throttle_scale
        Y *= self.throttle_scale
        H *= self.throttle_scale

        # --------- mix & publish ---------
        cmd = self.mix_like_keyboard(F=F, Y=Y, H=H)
        self.last_cmd = cmd
        self.publish_cmd(cmd)

    def _stop(self):
        if any(abs(v) > 1e-6 for v in self.last_cmd):
            self.last_cmd = [0.0] * 6
            self.publish_cmd(self.last_cmd)

def main():
    rclpy.init()
    node = JoyToThrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
