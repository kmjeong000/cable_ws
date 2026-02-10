#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import math
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class BlueROV2KeyboardTeleop(Node):
    """
    Keymap:
      w: forward
      s: backward
      a: left turn (CCW)
      d: right turn (CW)
      r: ascend
      f: descend
      space: stop (all zeros)
      q: quit
    """

    def __init__(self):
        super().__init__("keyboard_teleop_bluerov2")

        # Thruster command topics (your model uses cmd_thrust)
        self.pub = {}
        for i in range(1, 7):
            topic = f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust"
            self.pub[i] = self.create_publisher(Float64, topic, 10)

        # Pitch compensation from odometry 
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.prev_pitch = 0.0
        self.prev_pitch_time = time.time()

        # Gains (start values)
        self.kp_pitch = 20.0    # P gain
        self.kd_pitch = 5.0     # D gain (from numerical derivative)
        self.p_max = 10.0       # clamp for P_cmd

        self.odom_sub = self.create_subscription(
            Odometry,
            "/model/bluerov2/odometry",
            self.on_odom,
            10,
        )

        # Tuning
        self.step_xy = 15.0    # forward/back + yaw magnitude
        self.step_z = 15.0     # ascend/descend magnitude
        self.max_cmd = 30.0    # clamp output
        self.pitch_ref = None
        self._last_log_time = 0.0

        # “Hold-to-move” behavior: if no key seen within this, publish zeros
        self.key_timeout_s = 0.5

        self.last_key_time = time.time()
        self.last_cmd = [0.0] * 6  # t1..t6

        self.get_logger().info(
            "BlueROV2 Keyboard Teleop started.\n"
            "w/s: forward/back, a/d: yaw, r/f: ascend/descend, space: stop, q: quit"
        )

        # timer publishes at fixed rate (so it keeps applying force while key is held)
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)  # 30 Hz

        # terminal settings
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def quat_to_pitch(self, qx, qy, qz, qw) -> float:
        """
        Quaternion -> pitch(rad).
        Standard formula (ENU):
            pitch = asin(2*(w*y - z*x))
        """
        s = 2.0 * (qw * qy - qz * qx)
        s = max(-1.0, min(1.0, s))
        return math.asin(s)

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        pitch_now = self.quat_to_pitch(q.x, q.y, q.z, q.w)

        now = time.time()
        dt = now - self.prev_pitch_time
        if dt > 1e-4:
            # numerical derivative
            self.pitch_rate = (pitch_now - self.prev_pitch) / dt
            # clip derivative to avoid spikes
            self.pitch_rate = max(-5.0, min(5.0, self.pitch_rate))

        self.pitch = pitch_now
        self.prev_pitch = pitch_now
        self.prev_pitch_time = now

    def destroy_node(self):
        # restore terminal
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass
        super().destroy_node()

    def clamp(self, x: float) -> float:
        return max(-self.max_cmd, min(self.max_cmd, x))

    def mix(self, F: float, Y: float, H: float, P: float = 0.0):
        """
        Mixer based on your observed signs:
          thruster1 + : CW
          thruster2 + : CCW
          thruster3 + : CCW (also tends to pitch up)
          thruster4 + : CW  (also tends to pitch up; weaker)

        Horizontal:
          t1 = -F - Y - P
          t2 = -F + Y - P
          t3 = F + Y + P
          t4 = F - Y + P

        Vertical (you said + is descend, - is ascend):
          t5 = -H
          t6 = -H
        """
        bias = 2.0 if abs(F) > 1e-6 else 0.0
        bias *= 1.0 if F > 0 else -1.0

        t1 = -F - Y + bias
        t2 = -F + Y
        t3 = F + Y + bias
        t4 = F - Y
        t5 = -H
        t6 = -H
        return [self.clamp(v) for v in (t1, t2, t3, t4, t5, t6)]

    def read_key_nonblocking(self):
        # returns a single char if available, else None
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def publish_cmd(self, cmd):
        for i in range(1, 7):
            msg = Float64()
            msg.data = float(cmd[i - 1])
            self.pub[i].publish(msg)

    def on_timer(self):
        key = self.read_key_nonblocking()
        now = time.time()

        if key is not None:
            self.last_key_time = now

            # default: no motion
            F = 0.0
            Y = 0.0
            H = 0.0
            P_cmd = 0.0

            if key == "w":
                F = +self.step_xy
            elif key == "s":
                F = -self.step_xy
            elif key == "a":
                # left turn = CCW => Y positive in our convention
                Y = +self.step_xy
            elif key == "d":
                # right turn = CW => Y negative
                Y = -self.step_xy
            elif key == "r":
                # ascend => H positive (we invert in mix for thruster 5/6)
                H = +self.step_z
            elif key == "f":
                # descend
                H = -self.step_z
            elif key == " ":
                self.pitch_ref = None
                self.last_cmd = [0.0] * 6
                self.publish_cmd(self.last_cmd)
                return
            elif key == "q":
                self.last_cmd = [0.0] * 6
                self.publish_cmd(self.last_cmd)
                rclpy.shutdown()
                return
            else:
                # unknown key -> treat as no motion
                pass
            
            # Pitch compensation (PD)
            moving_for_pitch = (abs(F) > 1e-6) or (abs(H) > 1e-6)

            if moving_for_pitch:
                if self.pitch_ref is None:
                    self.pitch_ref = 0.0
                pitch_err = self.pitch - self.pitch_ref
                P_cmd = (self.kp_pitch * pitch_err + self.kd_pitch * self.pitch_rate)
                P_cmd = max(-self.p_max, min(self.p_max, P_cmd))
            else:
                P_cmd = 0.0
                self.pitch_ref = None
            
            self.last_cmd = self.mix(F=F, Y=Y, H=H, P=P_cmd)

            if now - self._last_log_time > 1.0:
                self.get_logger().info(
                    f"pitch={self.pitch:.3f} rad, prate={self.pitch_rate:.3f} rad/s, P_cmd={P_cmd:.2f}, "
                    f"F={F:.1f}, Y={Y:.1f}, H={H:.1f}, cmd={self.last_cmd}"
                )
                self._last_log_time = now

            self.publish_cmd(self.last_cmd)
            return

        # No key received this tick -> if timeout exceeded, stop
        if (now - self.last_key_time) > self.key_timeout_s:
            if any(abs(v) > 1e-6 for v in self.last_cmd):
                self.last_cmd = [0.0] * 6
                self.publish_cmd(self.last_cmd)

def main():
    rclpy.init()
    node = BlueROV2KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
