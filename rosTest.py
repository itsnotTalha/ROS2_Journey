import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading

class TeleopRelay(Node):
    def __init__(self):
        super().__init__('teleop_relay_node')

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_vel_pub = self.create_publisher(Twist, '/arm_teleop_vel', 10)
        self.indicator_pub = self.create_publisher(String, '/indicator', 10)

        # --- State Variables ---
        self.drive_active = False
        self.arm_active = False

        # --- Subscriptions ---
        # Note: We use specific topics. Remap these in your launch file or run command.
        self.drive_sub = self.create_subscription(Joy, '/joy/drive', self.drive_callback, 10)
        self.arm_sub = self.create_subscription(Joy, '/joy/arm', self.arm_callback, 10)

        # --- Controller Profiles (Mapping setup) ---
        self.profiles = {
            "ps4": {"lin": 1, "ang": 0, "lin2": 4, "ang2": 3},
            "xbox": {"lin": 1, "ang": 0, "lin2": 4, "ang2": 3},
            "logitech": {"lin": 1, "ang": 0, "twist": 2, "trigger": 0}
        }

    def start_drive(self):
        self.drive_active = True
        self.arm_active = False # Mutual exclusivity
        self.publish_indicator("Blue -> Manual Drive Mode")
        self.get_logger().info("System: DRIVE MODE ACTIVE")

    def start_arm(self):
        self.arm_active = True
        self.drive_active = False
        self.publish_indicator("Green -> Arm Control Mode")
        self.get_logger().info("System: ARM MODE ACTIVE")

    def shutdown_all(self):
        self.drive_active = False
        self.arm_active = False
        self.publish_indicator("RED -> All Systems Standby")
        self.get_logger().warn("System: ALL TELEOP DISABLED")

    def publish_indicator(self, text):
        msg = String()
        msg.data = text
        self.indicator_pub.publish(msg)

    def apply_deadzone(self, val, threshold=0.17):
        return val if abs(val) > threshold else 0.0

    def drive_callback(self, msg):
        if not self.drive_active:
            return
        
        # Logic for PS4/Xbox style dual-stick drive
        twist = Twist()
        # Mix axes 1/4 for linear and 0/3 for angular (as per your original code)
        twist.linear.x = self.apply_deadzone((msg.axes[1]/2) + (msg.axes[4]/2))
        twist.angular.z = self.apply_deadzone((msg.axes[0]/2) + (msg.axes[3]/2))
        
        self.cmd_vel_pub.publish(twist)

    def arm_callback(self, msg):
        if not self.arm_active:
            return
        
        # Example Arm Logic (Mapping to a different topic)
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Standard pitch
        twist.angular.z = msg.axes[0] # Standard yaw
        self.arm_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopRelay()

    # Spin in a background thread so input() doesn't block ROS callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            print("\n--- Rover Relay Control ---")
            print(f"Status: Drive [{'ON' if node.drive_active else 'OFF'}] | Arm [{'ON' if node.arm_active else 'OFF'}]")
            print("1. Start Drive Joy")
            print("2. Start Arm Joy")
            print("3. Stop All (Emergency Standby)")
            print("0. Exit Node")

            choice = input("Select Option: ")

            if choice == '1':
                node.start_drive()
            elif choice == '2':
                node.start_arm()
            elif choice == '3':
                node.shutdown_all()
            elif choice == '0':
                break
            else:
                print("Invalid input.")

    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()