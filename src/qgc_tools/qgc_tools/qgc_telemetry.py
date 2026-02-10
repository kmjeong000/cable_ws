#!/usr/bin/env python3
import math, time
from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure, Temperature

from pymavlink import mavutil

QGC_HOST = "host.docker.internal"
QGC_PORT = 14550

KST = ZoneInfo("Asia/Seoul")

def quat_to_yaw(qx, qy, qz, qw):
    # yaw (Z) from quaternion (x,y,z,w), ROS ENU
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class QGCTelemetry(Node):
    def __init__(self):
        super().__init__("qgc_telemetry")

        # --- parameters ---
        self.rho = 1025.0
        self.g = 9.80665

        # --- inputs ---
        self.sub_odom = self.create_subscription(
            Odometry, "/model/bluerov2/odometry", self.cb_odom, 10
        )

        # pressure / temperature topic
        self.sub_pressure = self.create_subscription(
          FluidPressure, "/model/bluerov2/sensor/pressure/pressure", self.cb_pressure, 10
        )
        self.sub_temp = self.create_subscription(
          Temperature, "/model/bluerov2/sensor/temperature/temperature", self.cb_temp, 10
        )

        # thruster cmd
        self.thr = [0.0] * 6
        self.thr_last_t = [None] * 6
        for i in range(6):
            topic = f"/model/bluerov2/joint/thruster{i+1}_joint/cmd_thrust"
            self.create_subscription(Float64, topic, self._mk_thr_cb(i), 10)

        # --- state ---
        self.last_yaw = None

        # depth
        self.p0 = None
        self.last_pressure = None
        self.last_depth = None

        self.last_temp_c = None

        # --- mavlink out to QGC ---
        self.m = mavutil.mavlink_connection(f"udpout:{QGC_HOST}:{QGC_PORT}")
        self.m.mav.srcSystem = 42
        self.m.mav.srcComponent = 1

        self.last_hb = 0.0
        self.last_send = 0.0

        # 10Hz tick
        self.timer = self.create_timer(0.1, self.tick)

    def _mk_thr_cb(self, idx):
        def cb(msg: Float64):
            self.thr[idx] = float(msg.data)
            self.thr_last_t[idx] = time.time()
        return cb

    def cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.last_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def cb_pressure(self, msg: FluidPressure):
        p = float(msg.fluid_pressure)
        self.last_pressure = p

        if self.p0 is None:
            self.p0 = p
            self.last_depth = 0.0
            self.get_logger().info(f"Pressure calibrated: p0={self.p0:.1f} Pa")
            return

        self.last_depth = max(0.0, (p - self.p0) / (self.rho * self.g))

    def cb_temp(self, msg: Temperature):
        self.last_temp_c = float(msg.temperature)

    def _thr_mean(self):
        now = time.time()
        if any(t is None or (now - t) > 1.0 for t in self.thr_last_t):
            return None
        return sum(abs(x) for x in self.thr) / 6.0      

    def tick(self):
        now = time.time()

        # 1) Heartbeat 1Hz (QGC가 '기체 있음'으로 인식)
        if now - self.last_hb >= 1.0:
            self.m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_SUBMARINE,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )
            self.last_hb = now

        # 2) 텍스트 2Hz
        if now - self.last_send < 0.5:
            return
        self.last_send = now

        kst = datetime.now(KST).strftime("%H:%M:%S")

        yaw_deg = math.degrees(self.last_yaw) if self.last_yaw is not None else float("nan")
        depth_m = self.last_depth if self.last_depth is not None else float("nan")
        temp_c = self.last_temp_c if self.last_temp_c is not None else float("nan")

        thr_mean = self._thr_mean()
        
        if thr_mean is None:
            thr_str = "STALE"
        else:
            thr_str = f"{thr_mean:.2f}"   
            
        text = f"{kst} hdg={yaw_deg:5.1f} dep={depth_m:4.1f} T={temp_c:4.1f} thr={thr_str}"
        self.m.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text.encode("utf-8")[:50]
        )

def main():
    rclpy.init()
    node = QGCTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
