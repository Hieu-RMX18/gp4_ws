#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import math
import numpy as np

class CameraImuRpyNode(Node):
    def __init__(self):
        super().__init__('camera_imu_rpy_node')
        
        # Declare topic parameter, default to RealSense IMU topic
        self.declare_parameter('imu_topic', '/camera/imu')
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to IMU topic: {imu_topic}")
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        
        # Check if quaternion is valid (not all zeros)
        if abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6 and abs(q.w) < 1e-6:
            # Fallback to calculating Roll and Pitch from accelerometer (gravity)
            # This assumes the camera is relatively static or moving slowly.
            accel = msg.linear_acceleration
            
            # Roll (rotation around X axis)
            roll = math.atan2(accel.y, accel.z)
            
            # Pitch (rotation around Y axis)
            pitch = math.atan2(-accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z))
            
            yaw = 0.0 # Yaw cannot be determined solely from accelerometer
            
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = 0.0
            
            self.get_logger().info(f"[Accel Fallback] Roll: {roll_deg:6.2f} deg | Pitch: {pitch_deg:6.2f} deg | Yaw: Unknown", throttle_duration_sec=0.5)
        else:
            roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(f"Roll: {roll_deg:6.2f} deg | Pitch: {pitch_deg:6.2f} deg | Yaw: {yaw_deg:6.2f} deg", throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = CameraImuRpyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
