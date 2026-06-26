#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import tf2_ros

class TcpPathTracker(Node):
    def __init__(self):
        super().__init__('tcp_path_tracker')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'tool0')
        self.declare_parameter('update_rate', 10.0) # Hz
        self.declare_parameter('max_points', 5000)
        
        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_frame = self.get_parameter('tcp_frame').value
        update_rate = self.get_parameter('update_rate').value
        self.max_points = self.get_parameter('max_points').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        self.path_pub = self.create_publisher(Path, 'tcp_path', 10)
        
        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.base_frame
        
        # Timer
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        self.get_logger().info(f"TCP Path Tracker started: {self.base_frame} -> {self.tcp_frame}")

    def timer_callback(self):
        try:
            # Look up transform
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.tcp_frame, 
                rclpy.time.Time()
            )
            
            # Create pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.base_frame
            
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            
            pose.pose.orientation = trans.transform.rotation
            
            # Append to path
            self.path_msg.poses.append(pose)
            
            # Keep max points (optional)
            if len(self.path_msg.poses) > self.max_points:
                self.path_msg.poses.pop(0)
                
            self.path_msg.header.stamp = pose.header.stamp
            
            # Publish
            self.path_pub.publish(self.path_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"TF exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TcpPathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
