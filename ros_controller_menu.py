#!/usr/bin/env python3
"""
ROS Controller Configuration Menu
- Manual controller selection with MAC address identification
- Separate DRIVE and ARM control systems (no conflicts)
- Modular button/axis mappings for easy customization
- ROS2 integration via joy_node subprocess
- Colorful & clean UI

Architecture:
- This program spawns joy_node processes internally
- joy_node lifecycle is controlled by user toggle actions
- ConditionalTeleopNode subscribes to /joy/drive and /joy/arm
- Publishes processed commands to /buswala and /aram
"""

import curses
from curses import wrapper
import os
import glob
import threading
import time
import subprocess
import signal

# Try to import ROS2 - graceful fallback if not available
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    pass


# ================================================================================
# MODULAR BUTTON/AXIS MAPPING CONFIGURATION
# Edit these dictionaries to change controller behavior
# ================================================================================

# Controller profiles for detection
CONTROLLER_PROFILES = {
    "Xbox-360 Controller": {"buttons": 11, "axes": 8},
    "PS4 Controller": {"buttons": 13, "axes": 8},
    "Logitech X-3D Pro": {"buttons": 12, "axes": 6},
}

# -------------------- DRIVE CONTROLLER MAPPINGS --------------------
# Customize which buttons/axes control drive functions
DRIVE_MAPPINGS = {
    "Xbox-360 Controller": {
        # Mode switching
        "manual_mode_button": 2,      # X button -> enable manual
        "auto_mode_button": 1,        # B button -> enable auto
        # Movement axes
        "linear_axis_1": 1,           # Left stick Y
        "linear_axis_2": 4,           # Right stick Y
        "angular_axis_1": 0,          # Left stick X
        "angular_axis_2": 3,          # Right stick X
        # Combine both sticks (each contributes 50%)
        "linear_scale": 0.5,
        "angular_scale": 0.5,
        "deadzone": 0.1,
    },
    "PS4 Controller": {
        "manual_mode_button": 3,      # Square button -> enable manual
        "auto_mode_button": 1,        # Circle button -> enable auto
        "linear_axis_1": 1,
        "linear_axis_2": 4,
        "angular_axis_1": 0,
        "angular_axis_2": 3,
        "linear_scale": 0.5,
        "angular_scale": 0.5,
        "deadzone": 0.17,
    },
    "Logitech X-3D Pro": {
        "manual_mode_button": 1,      # Button 1 -> enable manual
        "auto_mode_button": 2,        # Button 2 -> enable auto
        "emergency_stop_button": 0,   # Trigger -> emergency stop
        "linear_axis_1": 1,           # Y axis
        "angular_axis_1": 0,          # X axis
        "rotate_axis": 2,             # Twist axis for 360 rotate
        "linear_scale": 1.0,
        "angular_scale": 1.0,
        "deadzone": 0.1,
        "rotate_threshold": 0.17,
    },
    # Default fallback for unknown controllers
    "default": {
        "manual_mode_button": 0,
        "auto_mode_button": 1,
        "linear_axis_1": 1,
        "angular_axis_1": 0,
        "linear_scale": 1.0,
        "angular_scale": 1.0,
        "deadzone": 0.1,
    },
}

# -------------------- ARM CONTROLLER MAPPINGS --------------------
# Customize which buttons/axes control arm functions
ARM_MAPPINGS = {
    "Xbox-360 Controller": {
        # Arm movement axes
        "arm_x_axis": 1,              # Left stick Y -> forward/back
        "arm_y_axis": 0,              # Left stick X -> left/right
        "arm_z_axis": 4,              # Right stick Y -> up/down
        "wrist_roll_axis": 3,         # Right stick X -> wrist roll
        "wrist_pitch_axis": 2,        # Left trigger -> wrist pitch
        # Gripper buttons
        "gripper_open_button": 4,     # LB
        "gripper_close_button": 5,    # RB
        # Scaling
        "arm_scale": 1.0,
        "deadzone": 0.1,
    },
    "PS4 Controller": {
        "arm_x_axis": 1,
        "arm_y_axis": 0,
        "arm_z_axis": 4,
        "wrist_roll_axis": 3,
        "wrist_pitch_axis": 2,
        "gripper_open_button": 4,     # L1
        "gripper_close_button": 5,    # R1
        "arm_scale": 1.0,
        "deadzone": 0.17,
    },
    "Logitech X-3D Pro": {
        "arm_x_axis": 1,              # Y axis
        "arm_y_axis": 0,              # X axis
        "arm_z_axis": 3,              # Throttle
        "wrist_roll_axis": 2,         # Twist
        "gripper_open_button": 3,
        "gripper_close_button": 4,
        "arm_scale": 1.0,
        "deadzone": 0.1,
    },
    "default": {
        "arm_x_axis": 1,
        "arm_y_axis": 0,
        "arm_z_axis": 4,
        "wrist_roll_axis": 3,
        "wrist_pitch_axis": 2,
        "gripper_open_button": 4,
        "gripper_close_button": 5,
        "arm_scale": 1.0,
        "deadzone": 0.1,
    },
}

# -------------------- CONTROLLER OWNER NAMES --------------------
# Map MAC addresses to custom owner names for fun!
CONTROLLER_OWNERS = {
    "50:ee:32:04:32:53": "Abid Hossain",
    "84:30:95:41:0e:74": "Fahim Hafiz",
}


# ================================================================================
# CONTROLLER DETECTION
# ================================================================================

def get_controller_mac(js_path):
    """
    Get MAC address of a wired controller via its input device.
    For USB controllers, this extracts a unique identifier from the device path.
    """
    try:
        # Get the device name (e.g., js0)
        js_name = os.path.basename(js_path)
        
        # Try to get the physical path which contains unique identifiers
        phys_path = f"/sys/class/input/{js_name}/device/phys"
        if os.path.exists(phys_path):
            with open(phys_path) as f:
                phys = f.read().strip()
                # Physical path often contains USB path like "usb-0000:00:14.0-1/input0"
                # We can use this as a unique identifier
                if phys:
                    return phys
        
        # Alternative: try to get uniq (unique ID) if available
        uniq_path = f"/sys/class/input/{js_name}/device/uniq"
        if os.path.exists(uniq_path):
            with open(uniq_path) as f:
                uniq = f.read().strip()
                if uniq:
                    return uniq
        
        # Fallback: use device path + vendor:product as identifier
        vendor_path = f"/sys/class/input/{js_name}/device/id/vendor"
        product_path = f"/sys/class/input/{js_name}/device/id/product"
        if os.path.exists(vendor_path) and os.path.exists(product_path):
            with open(vendor_path) as f:
                vendor = f.read().strip()
            with open(product_path) as f:
                product = f.read().strip()
            return f"{vendor}:{product}@{js_path}"
        
        return "N/A"
    except:
        return "N/A"


def find_joysticks():
    """Find all connected joystick devices with MAC addresses"""
    devices = []
    for js in sorted(glob.glob("/dev/input/js*")):
        name = "Unknown"
        try:
            with open(f"/sys/class/input/{os.path.basename(js)}/device/name") as f:
                name = f.read().strip()
        except:
            pass
        
        mac = get_controller_mac(js)
        owner = CONTROLLER_OWNERS.get(mac, None)  # Get owner name if MAC matches
        devices.append({"path": js, "name": name, "mac": mac, "owner": owner})
    return devices


# ================================================================================
# ROS2 CONDITIONAL TELEOP NODE
# ================================================================================

if ROS2_AVAILABLE:
    class ConditionalTeleopNode(Node):
        """
        Single ROS2 Node for both DRIVE and ARM control.
        
        - Subscribes to /joy/drive and /joy/arm (dynamically created)
        - Publishes to /buswala (drive) and /aram (arm)
        - Uses dummy node pattern for manual/auto mode switching
        - Detects controller model from Joy message button/axis counts
        """

        def __init__(self):
            super().__init__('conditional_teleop_node')

            # ---- Publishers (always exist) ----
            self.buswala_pub = self.create_publisher(Twist, '/buswala', 10)
            self.aram_pub = self.create_publisher(Twist, '/aram', 10)
            self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
            self.indicator_pub = self.create_publisher(String, '/indicator', 10)

            # ---- Dynamic subscriptions (created on toggle) ----
            self.drive_joy_sub = None
            self.arm_joy_sub = None

            # ---- State ----
            self.drive_enabled = False
            self.arm_enabled = False
            self.last_drive_joy_time = None
            self.last_arm_joy_time = None

            # ---- Dummy node pattern for mode switching ----
            self.drive_dummy_node = None
            self.drive_send_msg = False
            self.drive_controller_model = None

            self.arm_dummy_node = None
            self.arm_send_msg = False
            self.arm_controller_model = None

            # ---- Safety timer (20Hz) ----
            self.safety_timer = self.create_timer(0.05, self._safety_check)

        # ================== DRIVE CONTROL ==================

        def enable_drive(self):
            """Create subscription to /joy/drive"""
            if self.drive_joy_sub is None:
                self.drive_joy_sub = self.create_subscription(
                    Joy, '/joy/drive', self._drive_joy_callback, 10)
                self.drive_enabled = True
                self.last_drive_joy_time = time.time()

        def disable_drive(self):
            """Destroy subscription to /joy/drive and publish zero"""
            if self.drive_joy_sub is not None:
                self.destroy_subscription(self.drive_joy_sub)
                self.drive_joy_sub = None
            self.drive_enabled = False
            self._destroy_drive_dummy_node()
            self._publish_zero_buswala()

        def _drive_joy_callback(self, msg: Joy):
            """Process /joy/drive messages"""
            self.last_drive_joy_time = time.time()

            # Detect controller model from message
            self._detect_drive_controller_model(msg)

            # Mode switching with dummy node pattern
            if self.drive_controller_model == "Xbox-360 Controller":
                if msg.buttons[2] == 1 and self.drive_dummy_node is None:
                    self.drive_send_msg = True
                    self._create_drive_dummy_node()
                elif msg.buttons[1] == 1 and self.drive_dummy_node is not None:
                    self.drive_send_msg = False
                    self._destroy_drive_dummy_node()

            elif self.drive_controller_model == "PS4 Controller":
                if msg.buttons[3] == 1 and self.drive_dummy_node is None:
                    self.drive_send_msg = True
                    self._create_drive_dummy_node()
                elif msg.buttons[1] == 1 and self.drive_dummy_node is not None:
                    self.drive_send_msg = False
                    self._destroy_drive_dummy_node()

            elif self.drive_controller_model == "Logitech X-3D Pro":
                if msg.buttons[1] == 1 and self.drive_dummy_node is None:
                    self.drive_send_msg = True
                    self._create_drive_dummy_node()
                elif msg.buttons[2] == 1 and self.drive_dummy_node is not None:
                    self.drive_send_msg = False
                    self._destroy_drive_dummy_node()

            # Publish twist only in manual mode
            if self.drive_send_msg:
                self._publish_drive_twist(msg)

        def _detect_drive_controller_model(self, msg: Joy):
            """Detect controller model from button/axis counts"""
            num_buttons = len(msg.buttons)
            num_axes = len(msg.axes)

            if num_buttons == 11 and num_axes == 8:
                self.drive_controller_model = "Xbox-360 Controller"
            elif num_buttons == 13 and num_axes == 8:
                self.drive_controller_model = "PS4 Controller"
            elif num_buttons == 12 and num_axes == 6:
                self.drive_controller_model = "Logitech X-3D Pro"
            else:
                self.drive_controller_model = "Unknown"

        def _create_drive_dummy_node(self):
            """Create dummy node to indicate manual drive mode"""
            self.drive_dummy_node = rclpy.create_node('teleop_drive_is_on')
            indicator = String()
            indicator.data = "Blue -> Manual Mode"
            self.indicator_pub.publish(indicator)

        def _destroy_drive_dummy_node(self):
            """Destroy dummy node to indicate autonomous mode"""
            if self.drive_dummy_node is not None:
                self.drive_dummy_node.destroy_node()
                self.drive_dummy_node = None
                self.drive_send_msg = False
                indicator = String()
                indicator.data = "RED -> Autonomous Mode"
                self.indicator_pub.publish(indicator)

        def _publish_drive_twist(self, joy: Joy):
            """Publish processed Twist to /buswala"""
            twist = Twist()

            if self.drive_controller_model == "Xbox-360 Controller":
                twist.linear.x = (joy.axes[1] / 2) + (joy.axes[4] / 2)
                twist.angular.z = (joy.axes[0] / 2) + (joy.axes[3] / 2)

            elif self.drive_controller_model == "PS4 Controller":
                twist.linear.x = (joy.axes[1] / 2) + (joy.axes[4] / 2)
                twist.angular.z = (joy.axes[0] / 2) + (joy.axes[3] / 2)
                # Apply deadzone 0.17
                if abs(joy.axes[0]) < 0.17:
                    twist.angular.z -= joy.axes[0] / 2
                if abs(joy.axes[3]) < 0.17:
                    twist.angular.z -= joy.axes[3] / 2
                if abs(joy.axes[1]) < 0.17:
                    twist.linear.x -= joy.axes[1] / 2
                if abs(joy.axes[4]) < 0.17:
                    twist.linear.x -= joy.axes[4] / 2

            elif self.drive_controller_model == "Logitech X-3D Pro":
                if joy.buttons[0]:  # Emergency stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                # 360 rotate mode
                elif abs(joy.axes[2]) > 0.17 and abs(joy.axes[1]) <= 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = joy.axes[2]
                else:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]
                    # Apply deadzone 0.1
                    if abs(twist.angular.z) < 0.1:
                        twist.angular.z = 0.0

            else:
                # Default: use axes 1 and 0
                twist.linear.x = joy.axes[1] if len(joy.axes) > 1 else 0.0
                twist.angular.z = joy.axes[0] if len(joy.axes) > 0 else 0.0

            self.buswala_pub.publish(twist)

        def _publish_zero_buswala(self):
            """Publish zero Twist to /buswala"""
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.buswala_pub.publish(twist)

        # ================== ARM CONTROL ==================

        def enable_arm(self):
            """Create subscription to /joy/arm"""
            if self.arm_joy_sub is None:
                self.arm_joy_sub = self.create_subscription(
                    Joy, '/joy/arm', self._arm_joy_callback, 10)
                self.arm_enabled = True
                self.last_arm_joy_time = time.time()

        def disable_arm(self):
            """Destroy subscription to /joy/arm and publish zero"""
            if self.arm_joy_sub is not None:
                self.destroy_subscription(self.arm_joy_sub)
                self.arm_joy_sub = None
            self.arm_enabled = False
            self._destroy_arm_dummy_node()
            self._publish_zero_aram()

        def _arm_joy_callback(self, msg: Joy):
            """Process /joy/arm messages"""
            self.last_arm_joy_time = time.time()

            # Detect controller model
            self._detect_arm_controller_model(msg)

            # ARM always publishes when enabled (no manual/auto toggle for arm)
            # But we can add mode switching if needed
            self._publish_arm_twist(msg)
            self._handle_gripper(msg)

        def _detect_arm_controller_model(self, msg: Joy):
            """Detect controller model from button/axis counts"""
            num_buttons = len(msg.buttons)
            num_axes = len(msg.axes)

            if num_buttons == 11 and num_axes == 8:
                self.arm_controller_model = "Xbox-360 Controller"
            elif num_buttons == 13 and num_axes == 8:
                self.arm_controller_model = "PS4 Controller"
            elif num_buttons == 12 and num_axes == 6:
                self.arm_controller_model = "Logitech X-3D Pro"
            else:
                self.arm_controller_model = "Unknown"

        def _create_arm_dummy_node(self):
            """Create dummy node to indicate arm control active"""
            self.arm_dummy_node = rclpy.create_node('teleop_arm_is_on')

        def _destroy_arm_dummy_node(self):
            """Destroy arm dummy node"""
            if self.arm_dummy_node is not None:
                self.arm_dummy_node.destroy_node()
                self.arm_dummy_node = None
                self.arm_send_msg = False

        def _publish_arm_twist(self, joy: Joy):
            """Publish processed Twist to /aram"""
            twist = Twist()
            mapping = ARM_MAPPINGS.get(self.arm_controller_model, ARM_MAPPINGS["default"])
            deadzone = mapping.get("deadzone", 0.1)
            scale = mapping.get("arm_scale", 1.0)

            def apply_deadzone(val, thresh):
                return 0.0 if abs(val) < thresh else val

            # Get axes safely
            def get_axis(idx):
                return joy.axes[idx] if idx < len(joy.axes) else 0.0

            twist.linear.x = apply_deadzone(get_axis(mapping.get("arm_x_axis", 1)), deadzone) * scale
            twist.linear.y = apply_deadzone(get_axis(mapping.get("arm_y_axis", 0)), deadzone) * scale
            twist.linear.z = apply_deadzone(get_axis(mapping.get("arm_z_axis", 4)), deadzone) * scale
            twist.angular.x = apply_deadzone(get_axis(mapping.get("wrist_roll_axis", 3)), deadzone) * scale
            if "wrist_pitch_axis" in mapping:
                twist.angular.y = apply_deadzone(get_axis(mapping["wrist_pitch_axis"]), deadzone) * scale

            self.aram_pub.publish(twist)

        def _handle_gripper(self, joy: Joy):
            """Handle gripper open/close commands"""
            mapping = ARM_MAPPINGS.get(self.arm_controller_model, ARM_MAPPINGS["default"])
            open_btn = mapping.get("gripper_open_button")
            close_btn = mapping.get("gripper_close_button")

            def get_button(idx):
                return joy.buttons[idx] if idx < len(joy.buttons) else 0

            if open_btn is not None and get_button(open_btn):
                msg = String()
                msg.data = "open"
                self.gripper_pub.publish(msg)
            elif close_btn is not None and get_button(close_btn):
                msg = String()
                msg.data = "close"
                self.gripper_pub.publish(msg)

        def _publish_zero_aram(self):
            """Publish zero Twist to /aram"""
            twist = Twist()
            self.aram_pub.publish(twist)

        # ================== SAFETY ==================

        def _safety_check(self):
            """Safety timer callback - publish zero if no Joy msg for > 0.5s"""
            now = time.time()

            if self.drive_enabled and self.drive_send_msg:
                if self.last_drive_joy_time and (now - self.last_drive_joy_time) > 0.5:
                    self._publish_zero_buswala()

            if self.arm_enabled:
                if self.last_arm_joy_time and (now - self.last_arm_joy_time) > 0.5:
                    self._publish_zero_aram()

        # ================== CLEANUP ==================

        def cleanup(self):
            """Full cleanup on shutdown"""
            self.disable_drive()
            self.disable_arm()
            if self.safety_timer:
                self.safety_timer.cancel()
                self.destroy_timer(self.safety_timer)

        def is_manual_mode(self):
            """Return whether drive is in manual mode (for UI)"""
            return self.drive_dummy_node is not None


# ---------------- MENU ----------------
class ControllerMenu:
    def __init__(self):
        self.controllers = []
        self.drive = None
        self.arm = None
        self.drive_on = False
        self.arm_on = False
        self.selection = 0

        # joy_node subprocess handles
        self.drive_joy_process = None
        self.arm_joy_process = None

        # ROS2 node
        self.teleop_node = None
        self.ros_spin_thread = None
        self.ros_initialized = False

        self.menu = [
            "Select DRIVE Controller",
            "Select ARM Controller",
            "Toggle DRIVE ROS Node",
            "Toggle ARM ROS Node",
            "Exit"
        ]

    def _init_ros(self):
        """Initialize ROS2 if available"""
        if not ROS2_AVAILABLE:
            return False
        if not self.ros_initialized:
            try:
                rclpy.init()
                self.ros_initialized = True
            except:
                pass
        return self.ros_initialized

    def _shutdown_ros(self):
        """Shutdown ROS2"""
        if self.ros_initialized:
            try:
                rclpy.shutdown()
            except:
                pass
            self.ros_initialized = False

    def _start_ros_spin_thread(self):
        """Start a thread to spin ROS2 nodes"""
        if self.ros_spin_thread is None or not self.ros_spin_thread.is_alive():
            self.ros_spin_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self.ros_spin_thread.start()

    def _ros_spin_loop(self):
        """Spin ROS2 teleop node in background"""
        while self.ros_initialized:
            try:
                if self.teleop_node:
                    rclpy.spin_once(self.teleop_node, timeout_sec=0.01)
                time.sleep(0.01)
            except:
                break

    def _ensure_teleop_node(self):
        """Ensure the ConditionalTeleopNode exists"""
        if self.teleop_node is None:
            self.teleop_node = ConditionalTeleopNode()
            self._start_ros_spin_thread()
        return True

    def _launch_joy_node(self, device_path, topic_remap):
        """
        Launch joy_node as subprocess.
        
        Args:
            device_path: /dev/input/jsX
            topic_remap: e.g., '/joy/drive' or '/joy/arm'
        
        Returns:
            subprocess.Popen handle or None
        """
        try:
            cmd = [
                'ros2', 'run', 'joy', 'joy_node',
                '--ros-args',
                '-p', f'dev:={device_path}',
                '-r', f'joy:={topic_remap}'
            ]
            # Launch as background process, suppress output
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            return process
        except Exception:
            return None

    def _terminate_joy_node(self, process):
        """Terminate a joy_node subprocess cleanly"""
        if process is None:
            return
        try:
            # Send SIGTERM to the process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2.0)
        except Exception:
            try:
                # Force kill if graceful termination fails
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1.0)
            except Exception:
                pass

    def _start_drive_ros(self):
        """Start DRIVE: launch joy_node subprocess and enable subscription"""
        if not ROS2_AVAILABLE:
            return False

        if not self.drive:
            return False

        if not self._init_ros():
            return False

        # Ensure teleop node exists
        if not self._ensure_teleop_node():
            return False

        # Launch joy_node for drive controller
        self.drive_joy_process = self._launch_joy_node(
            self.drive["path"], '/joy/drive')
        
        if self.drive_joy_process is None:
            return False

        # Give joy_node time to start
        time.sleep(0.3)

        # Enable drive subscription in teleop node
        self.teleop_node.enable_drive()
        return True

    def _stop_drive_ros(self):
        """Stop DRIVE: terminate joy_node and disable subscription"""
        # Disable subscription first (publishes zero)
        if self.teleop_node:
            self.teleop_node.disable_drive()

        # Terminate joy_node subprocess
        self._terminate_joy_node(self.drive_joy_process)
        self.drive_joy_process = None

    def _start_arm_ros(self):
        """Start ARM: launch joy_node subprocess and enable subscription"""
        if not ROS2_AVAILABLE:
            return False

        if not self.arm:
            return False

        if not self._init_ros():
            return False

        # Ensure teleop node exists
        if not self._ensure_teleop_node():
            return False

        # Launch joy_node for arm controller
        self.arm_joy_process = self._launch_joy_node(
            self.arm["path"], '/joy/arm')
        
        if self.arm_joy_process is None:
            return False

        # Give joy_node time to start
        time.sleep(0.3)

        # Enable arm subscription in teleop node
        self.teleop_node.enable_arm()
        return True

    def _stop_arm_ros(self):
        """Stop ARM: terminate joy_node and disable subscription"""
        # Disable subscription first (publishes zero)
        if self.teleop_node:
            self.teleop_node.disable_arm()

        # Terminate joy_node subprocess
        self._terminate_joy_node(self.arm_joy_process)
        self.arm_joy_process = None

    # -------- CONTROLLER REFRESH (NO AUTO-ASSIGN) --------
    def refresh_controllers(self):
        """Refresh controller list but do NOT auto-assign.
        User must manually select controllers.
        """
        self.controllers = find_joysticks()

        # Check if selected controllers are still connected
        if self.drive and self.drive not in self.controllers:
            # Find by MAC if controller reconnected on different path
            for ctrl in self.controllers:
                if ctrl["mac"] == self.drive["mac"]:
                    self.drive = ctrl
                    break
            else:
                self.drive = None

        if self.arm and self.arm not in self.controllers:
            for ctrl in self.controllers:
                if ctrl["mac"] == self.arm["mac"]:
                    self.arm = ctrl
                    break
            else:
                self.arm = None

    # ---------------- DRAW ----------------
    def draw(self, stdscr):
        stdscr.clear()
        h, w = stdscr.getmaxyx()
        mid = w // 2
        self.refresh_controllers()

        stdscr.addstr(1, 2, "ROS CONTROLLER CONFIGURATION", curses.A_BOLD | curses.color_pair(4))

        # ROS2 availability indicator
        status_text = "[ROS2 OK]" if ROS2_AVAILABLE else "[ROS2 N/A]"
        status_color = curses.color_pair(2) if ROS2_AVAILABLE else curses.color_pair(1)
        stdscr.addstr(1, 35, status_text, status_color)

        # DRIVE Info
        y = 3
        stdscr.addstr(y, 2, "DRIVE", curses.A_BOLD | curses.color_pair(3))
        if self.drive:
            owner = self.drive.get("owner", "Unknown")
            try:
                stdscr.addstr(y + 1, 4, f"[P] {owner}", curses.color_pair(2) | curses.A_BOLD)
                stdscr.addstr(y + 2, 4, f"MAC: {self.drive['mac'][:w-10]}", curses.color_pair(4))
            except curses.error:
                pass
        else:
            stdscr.addstr(y + 1, 4, "Not selected", curses.color_pair(1))
        
        d_ros = "* PUBLISHING" if self.drive_on and self.teleop_node else "o INACTIVE"
        try:
            stdscr.addstr(y + 4, 4, f"ROS: {d_ros}", curses.color_pair(2 if "PUBLISHING" in d_ros else 1))
        except curses.error:
            pass

        # ARM Info
        y = 9
        stdscr.addstr(y, 2, "ARM", curses.A_BOLD | curses.color_pair(3))
        if self.arm:
            owner = self.arm.get("owner", "Unknown")
            try:
                stdscr.addstr(y + 1, 4, f"[P] {owner}", curses.color_pair(2) | curses.A_BOLD)
                stdscr.addstr(y + 2, 4, f"MAC: {self.arm['mac'][:w-10]}", curses.color_pair(4))
            except curses.error:
                pass
        else:
            stdscr.addstr(y + 1, 4, "Not selected", curses.color_pair(1))

        a_ros = "* PUBLISHING" if self.arm_on and self.teleop_node else "o INACTIVE"
        try:
            stdscr.addstr(y + 4, 4, f"ROS: {a_ros}", curses.color_pair(2 if "PUBLISHING" in a_ros else 1))
        except curses.error:
            pass

        # MENU
        y = 15
        for i, item in enumerate(self.menu):
            if i == self.selection:
                stdscr.attron(curses.A_REVERSE)
            if y + i < h - 1:
                stdscr.addstr(y + i, 2, f" {item} ")
            if i == self.selection:
                stdscr.attroff(curses.A_REVERSE)

        # --- DASHBOARD SECTION (Mode Only) ---
        dash_y = h - 8  # Fixed position near the bottom
        
        # Only draw mode section if there's enough space
        if dash_y > y + len(self.menu) and dash_y > 0:
            # Check if the node exists to get the real mode
            is_manual = self.teleop_node.is_manual_mode() if self.teleop_node else False
            
            # DRIVE Side
            try:
                stdscr.addstr(dash_y, 4, "DRIVE", curses.A_BOLD)
                stdscr.addstr(dash_y + 1, 4, f"{'o' if is_manual else '*'} Autonomous")
                stdscr.addstr(dash_y + 2, 4, f"{'*' if is_manual else 'o'} Manual")

                # ARM Side (Update logic here if ARM also has modes)
                stdscr.addstr(dash_y, mid + 4, "ARM", curses.A_BOLD)
                stdscr.addstr(dash_y + 1, mid + 4, "o Autonomous")
                stdscr.addstr(dash_y + 2, mid + 4, "* Manual")
            except curses.error:
                pass  # Ignore curses errors from writing at edge of screen

        # Standard Footer
        try:
            stdscr.addstr(h - 2, 0, "Up/Down Navigate | ENTER Select | Q Quit".center(w)[:w-1], curses.A_REVERSE)
        except curses.error:
            pass
        stdscr.refresh()

    # ---------------- POPUP ----------------
    def popup_select(self, stdscr, title, assign, exclude_controller=None):
        """Popup for manual controller selection with MAC addresses.
        exclude_controller: controller dict that should be shown as unavailable (already in use)
        """
        sel = 0
        while True:
            # Refresh controller list
            self.controllers = find_joysticks()
            
            # Find available controllers (not already used by the other system)
            available_indices = []
            for i, d in enumerate(self.controllers):
                if exclude_controller and d.get("mac") == exclude_controller.get("mac"):
                    continue  # This controller is in use
                available_indices.append(i)
            
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))

            if not self.controllers:
                stdscr.addstr(4, 6, "No controllers found.", curses.color_pair(1))
                stdscr.addstr(5, 6, "Connect a controller and press R to refresh.", curses.color_pair(4))
            else:
                for i, d in enumerate(self.controllers):
                    # Check if this controller is already in use
                    is_unavailable = exclude_controller and d.get("mac") == exclude_controller.get("mac")
                    
                    # Show owner name if available, otherwise show controller name
                    owner = d.get("owner")
                    if owner:
                        line1 = f"[P] {owner} ({d['name']})"
                    else:
                        line1 = f"{d['name']}"
                    
                    # Add unavailable indicator
                    if is_unavailable:
                        line1 = f"[X] {line1} [IN USE]"
                    
                    line2 = f"  Path: {d['path']}  MAC: {d.get('mac', 'N/A')[:30]}"
                    
                    row = 4 + (i * 3)
                    
                    if is_unavailable:
                        # Show in red, not selectable
                        stdscr.addstr(row, 6, line1[:w-8], curses.color_pair(1))
                        stdscr.addstr(row + 1, 6, line2[:w-8], curses.color_pair(1))
                    else:
                        # Normal display
                        if i == sel:
                            stdscr.attron(curses.A_REVERSE)
                        stdscr.addstr(row, 6, line1[:w-8])
                        if i == sel:
                            stdscr.attroff(curses.A_REVERSE)
                        stdscr.addstr(row + 1, 6, line2[:w-8], curses.color_pair(4))

            stdscr.addstr(h - 2, 4, "↑↓ Navigate  ENTER Select  R Refresh  B Back", curses.A_REVERSE)
            stdscr.refresh()

            key = stdscr.getch()
            if key == curses.KEY_UP and available_indices:
                # Find previous available index
                current_pos = available_indices.index(sel) if sel in available_indices else 0
                new_pos = (current_pos - 1) % len(available_indices)
                sel = available_indices[new_pos]
            elif key == curses.KEY_DOWN and available_indices:
                # Find next available index
                current_pos = available_indices.index(sel) if sel in available_indices else 0
                new_pos = (current_pos + 1) % len(available_indices)
                sel = available_indices[new_pos]
            elif key in (10, 13) and available_indices and sel in available_indices:
                assign(self.controllers[sel])
                break
            elif key in (ord('r'), ord('R')):
                sel = available_indices[0] if available_indices else 0  # Reset to first available
            elif key in (ord('b'), ord('B')):
                break

    # ---------------- LOOP ----------------
    def run(self, stdscr):
        curses.curs_set(0)
        curses.start_color()

        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)

        try:
            while True:
                self.draw(stdscr)
                key = stdscr.getch()

                if key == curses.KEY_UP:
                    self.selection = (self.selection - 1) % len(self.menu)
                elif key == curses.KEY_DOWN:
                    self.selection = (self.selection + 1) % len(self.menu)
                elif key in (10, 13):
                    if self.selection == 0:
                        # Select DRIVE - exclude ARM controller if selected
                        self.popup_select(stdscr, "Select DRIVE Controller", 
                                         lambda d: setattr(self, "drive", d), 
                                         exclude_controller=self.arm)
                    elif self.selection == 1:
                        # Select ARM - exclude DRIVE controller if selected
                        self.popup_select(stdscr, "Select ARM Controller", 
                                         lambda d: setattr(self, "arm", d),
                                         exclude_controller=self.drive)
                    elif self.selection == 2:
                        # Toggle DRIVE ROS Node
                        if not self.drive_on:
                            if self.drive:
                                if self._start_drive_ros():
                                    self.drive_on = True
                                else:
                                    self._show_message(stdscr, "Error", "Failed to start ROS node. Check ROS2 installation.")
                            else:
                                self._show_message(stdscr, "No Controller", "Select a DRIVE controller first.")
                        else:
                            self._stop_drive_ros()
                            self.drive_on = False
                    elif self.selection == 3:
                        # Toggle ARM ROS Node
                        if not self.arm_on:
                            if self.arm:
                                if self._start_arm_ros():
                                    self.arm_on = True
                                else:
                                    self._show_message(stdscr, "Error", "Failed to start ROS node. Check ROS2 installation.")
                            else:
                                self._show_message(stdscr, "No Controller", "Select an ARM controller first.")
                        else:
                            self._stop_arm_ros()
                            self.arm_on = False
                    elif self.selection == 4:
                        break
                elif key in (ord('r'), ord('R')):
                    # Manual refresh
                    self.refresh_controllers()
                elif key in (ord('q'), ord('Q')):
                    break
        finally:
            # Cleanup - stop all joy_node processes and teleop node
            self.drive_on = False
            self.arm_on = False
            self._stop_drive_ros()
            self._stop_arm_ros()
            
            # Cleanup teleop node
            if self.teleop_node:
                try:
                    self.teleop_node.cleanup()
                    self.teleop_node.destroy_node()
                except:
                    pass
                self.teleop_node = None
            
            # Allow time for cleanup to propagate
            time.sleep(0.1)
            self._shutdown_ros()

    def _show_message(self, stdscr, title, message):
        stdscr.clear()
        stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 4, message)
        stdscr.addstr(6, 4, "Press any key to continue...")
        stdscr.refresh()
        stdscr.getch()


def main(stdscr):
    ControllerMenu().run(stdscr)


if __name__ == "__main__":
    wrapper(main)
