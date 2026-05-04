#!/usr/bin/env python3
"""Publish a nav_msgs/Path marker in RViz to visualize the TCP trail.

Subscribes to /yaskawa/joint_states, computes FK via /get_current_pose,
and publishes the accumulated path to /tcp_trail for RViz display.

Usage:
  ros2 run llm_gateway tcp_trail_marker
Or:
  python3 tcp_trail_marker.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interfaces.srv import GetCurrentPose


class TcpTrailMarker(Node):
    def __init__(self):
        super().__init__("tcp_trail_marker")

        self.path_pub = self.create_publisher(Path, "/tcp_trail", 10)

        # Periodically query current TCP pose via GET_POSE service
        self.get_pose_client = self.create_client(GetCurrentPose, "/get_current_pose")

        self.path = Path()
        self.path.header.frame_id = "base_link"

        # Poll at 10 Hz
        self.timer = self.create_timer(0.1, self.poll_pose)

        self.get_logger().info(
            "TCP trail marker started — visualize at /tcp_trail in RViz (add Path display)"
        )

    def poll_pose(self):
        if not self.get_pose_client.service_is_ready():
            return

        request = GetCurrentPose.Request()
        request.reference_frame = "base_link"

        future = self.get_pose_client.call_async(request)
        future.add_done_callback(self.on_pose_received)

    def on_pose_received(self, future):
        try:
            response = future.result()
        except Exception:
            return

        if not response or not response.success:
            return

        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_link"
        stamped.pose = response.current_pose

        self.path.header.stamp = stamped.header.stamp
        self.path.poses.append(stamped)

        # Keep last 2000 points to avoid OOM
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]

        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TcpTrailMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
