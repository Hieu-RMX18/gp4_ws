#!/usr/bin/env python3
"""Quick diagnostic: sample RGB values from the live pointcloud."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sys

class PCChecker(Node):
    def __init__(self):
        super().__init__('pc_rgb_checker')
        self.sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cb, 1)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        print(f"Fields: {[f.name for f in msg.fields]}")
        print(f"point_step={msg.point_step} width={msg.width} height={msg.height}")
        
        # Find rgb field
        rgb_field = None
        for f in msg.fields:
            if f.name in ('rgb', 'rgba'):
                rgb_field = f
                break
        
        if rgb_field is None:
            print("ERROR: No RGB field found!")
            rclpy.shutdown()
            return
        
        print(f"RGB field: name={rgb_field.name} offset={rgb_field.offset} "
              f"datatype={rgb_field.datatype} count={rgb_field.count}")
        
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        
        # Sample some points from the middle of the image
        mid_h = msg.height // 2
        mid_w = msg.width // 2
        
        # Sample a 20x20 patch from center
        samples = []
        for row in range(max(0, mid_h-10), min(msg.height, mid_h+10)):
            for col in range(max(0, mid_w-10), min(msg.width, mid_w+10)):
                idx = row * msg.width + col
                point_data = data[idx]
                
                # Read xyz
                x = np.frombuffer(point_data[0:4].tobytes(), dtype=np.float32)[0]
                y = np.frombuffer(point_data[4:8].tobytes(), dtype=np.float32)[0]
                z = np.frombuffer(point_data[8:12].tobytes(), dtype=np.float32)[0]
                
                # Read packed RGB at offset
                off = rgb_field.offset
                packed_bytes = point_data[off:off+4].tobytes()
                packed_float = np.frombuffer(packed_bytes, dtype=np.float32)[0]
                packed_uint = np.frombuffer(packed_bytes, dtype=np.uint32)[0]
                
                r = (packed_uint >> 16) & 0xFF
                g = (packed_uint >> 8) & 0xFF
                b = packed_uint & 0xFF
                
                if np.isfinite(x) and np.isfinite(z) and z > 0:
                    samples.append((x, y, z, r, g, b, packed_uint))
        
        print(f"\nSampled {len(samples)} valid points from center patch:")
        if samples:
            arr = np.array([(s[3], s[4], s[5]) for s in samples])
            print(f"  RGB mean: [{arr[:,0].mean():.1f}, {arr[:,1].mean():.1f}, {arr[:,2].mean():.1f}]")
            print(f"  RGB min:  [{arr[:,0].min()}, {arr[:,1].min()}, {arr[:,2].min()}]")
            print(f"  RGB max:  [{arr[:,0].max()}, {arr[:,1].max()}, {arr[:,2].max()}]")
            print(f"\n  First 10 samples (x,y,z, R,G,B, packed_hex):")
            for s in samples[:10]:
                print(f"    xyz=({s[0]:.3f},{s[1]:.3f},{s[2]:.3f}) "
                      f"RGB=({s[3]:3d},{s[4]:3d},{s[5]:3d}) packed=0x{s[6]:08X}")
            
            # Check if all RGB are very dark
            mean_brightness = arr.mean()
            if mean_brightness < 30:
                print(f"\n  *** WARNING: Mean brightness = {mean_brightness:.1f} — RGB data is VERY DARK!")
                print(f"  *** This likely means color texture is NOT being applied to the pointcloud.")
                print(f"  *** The 'No stream match' camera warning confirms this.")
            else:
                print(f"\n  Mean brightness = {mean_brightness:.1f} — RGB looks OK")
        
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PCChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
