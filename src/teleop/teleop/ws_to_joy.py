#!/usr/bin/env python3
import asyncio, json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import websockets

WS_HOST = "0.0.0.0"
WS_PORT = 5760

def fmt_list(name, arr):
    return f"{name}({len(arr)}): {arr}"

class JoyWS(Node):
    def __init__(self):
        super().__init__("joy_ws_server")
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.get_logger().info(f"WebSocket listening on ws://{WS_HOST}:{WS_PORT} -> /joy")

    def publish_payload(self, payload: dict):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "browser_gamepad"
        msg.axes = [float(x) for x in payload.get("axes", [])]
        msg.buttons = [int(b) for b in payload.get("buttons", [])]
        self.pub.publish(msg)

async def main_async():
    rclpy.init()
    node = JoyWS()

    async def handler(ws):
        node.get_logger().info("WS client connected")
        async for msg in ws:
            # 1) JSON parse
            try:
                payload = json.loads(msg)
            except Exception as e:
                node.get_logger().warn(f"bad json: {e}")
                continue

            # 2) optional debug
            # axes = payload.get("axes", [])
            # buttons = payload.get("buttons", [])
            # node.get_logger().info(
            #     "rx\n"
            #     f"  axes({len(axes)}): {axes}\n"
            #     f"  buttons({len(buttons)}): {buttons}",
            #     throttle_duration_sec=1.0
            # )

            # 3) publish Joy
            try:
                node.publish_payload(payload)
            except Exception as e:
                node.get_logger().error(f"publish failed: {e}")

    async with websockets.serve(handler, WS_HOST, WS_PORT):
        # spin + asyncio
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main_async())