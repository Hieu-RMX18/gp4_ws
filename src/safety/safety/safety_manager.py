import os
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

from .command_validator import CommandValidator
from .workspace_guard import WorkspaceGuard
from .execution_gate import ExecutionGate

class SafetyManager(Node):
    def __init__(self):
        super().__init__('safety_manager')
        
        # Load safety rules from config
        try:
            pkg_share = get_package_share_directory('safety')
            yaml_path = os.path.join(pkg_share, 'config', 'safety_rules.yaml')
            with open(yaml_path, 'r') as f:
                self.safety_rules = yaml.safe_load(f)
            self.get_logger().info("Loaded safety rules successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load safety rules: {e}")
            self.safety_rules = {}

        # Initialize internal modules
        self.validator = CommandValidator(self.safety_rules)
        self.guard = WorkspaceGuard(self.safety_rules)
        self.gate = ExecutionGate(self, self.validator, self.guard)

        # Emergency stop logic subscription
        self.status_sub = self.create_subscription(
            String,
            '/yaskawa/robot_status',
            self.status_callback,
            10
        )
        # Safety status publisher
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)
        
        self.get_logger().info("SafetyManager started and ready.")

    def status_callback(self, msg: String):
        # Trigger emergency stop logic if status has ERROR
        if "ERROR" in msg.data.upper():
            self.get_logger().error(f"EMERGENCY STOP CONDITION TRIGGERED: {msg.data}")
            self.publish_status("ERROR: Emergency stop triggered")
        else:
            self.publish_status("OK")

    def publish_status(self, status_msg: str):
        msg = String()
        msg.data = status_msg
        self.safety_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
