#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class QGCImage(Node):
    def __init__(self):
        super().__init__("qgc_image")

        self.topic = "/model/bluerov2/camera"
        self.host = "host.docker.internal"
        self.port = 5600
        self.fps = 15

        self.bridge = CvBridge()
        self.writer = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, self.topic, self.cb, qos)

        self.get_logger().info(f"[video] Sub: {self.topic}")
        self.get_logger().info(f"[video] Out: udp://{self.host}:{self.port} (H264 via openh264enc)")

    def _open(self, w: int, h: int):
        # OpenH264 encoder expects I420; we convert BGR -> I420 in the pipeline
        pipeline = (
            "appsrc is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={w},height={h},framerate={self.fps}/1 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "openh264enc bitrate=2000000 gop-size=30 rate-control=bitrate ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.host} port={self.port} sync=false"
        )
        self.writer = cv2.VideoWriter(
            pipeline, 
            cv2.CAP_GSTREAMER, 
            0, 
            self.fps, 
            (w, h), 
            True
        )
        if not self.writer.isOpened():
            raise RuntimeError("GStreamer pipeline open failed (openh264enc/rtph264pay missing?).")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        if self.writer is None:
            self._open(w,h)

        self.writer.write(frame)

def main():
    rclpy.init()
    node = QGCImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()