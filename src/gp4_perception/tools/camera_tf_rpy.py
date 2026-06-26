#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import math

class CameraTfRpyNode(Node):
    def __init__(self):
        super().__init__('camera_tf_rpy_node')
        
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Waiting for TF from {self.base_frame} to {self.camera_frame}...")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

    def timer_callback(self):
        try:
            # Look up transform from base_frame to camera_frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
            
            q = transform.transform.rotation
            roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Convert to degrees
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            self.get_logger().info(
                f"\n[TF Pose vs {self.base_frame}]\n"
                f"Translation (m): X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}\n"
                f"Rotation  (deg): Roll: {roll_deg:6.2f}, Pitch: {pitch_deg:6.2f}, Yaw: {yaw_deg:6.2f}"
            )
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"TF Lookup Error: {e}")
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"TF Connectivity Error: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF Extrapolation Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraTfRpyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
