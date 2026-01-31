#!/usr/bin/env python3
"""
ROS Controller Configuration Menu
- Real-time controller detection
- Auto-assign (same controller for DRIVE & ARM if only one exists)
- Manual selection available
- ROS2 integration for publishing controller data
- Colorful & clean UI
"""

import curses
from curses import wrapper
import os
import glob
import threading
import time
import struct
import select

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


# ---------------- CONTROLLER PROFILES ----------------
CONTROLLER_PROFILES = {
    "Xbox-360 Controller": {"buttons": 11, "axes": 8},
    "PS4 Controller": {"buttons": 13, "axes": 8},
    "Logitech X-3D Pro": {"buttons": 12, "axes": 6},
}


# ---------------- CONTROLLER DETECTION ----------------
def find_joysticks():
    devices = []
    for js in sorted(glob.glob("/dev/input/js*")):
        name = "Unknown"
        try:
            with open(f"/sys/class/input/{os.path.basename(js)}/device/name") as f:
                name = f.read().strip()
        except:
            pass
        devices.append({"path": js, "name": name})
    return devices


# ---------------- JOYSTICK READER ----------------
class JoystickReader:
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02

    def __init__(self, path):
        self.path = path
        self.running = False
        self.axes = {}
        self.buttons = {}
        self.num_axes = 0
        self.num_buttons = 0
        self.controller_model = "Unknown"
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _detect_model(self):
        """Detect controller model based on axes/buttons count"""
        for model, profile in CONTROLLER_PROFILES.items():
            if profile["buttons"] == self.num_buttons and profile["axes"] == self.num_axes:
                return model
        return f"Unknown ({self.num_buttons}B/{self.num_axes}A)"

    def _loop(self):
        try:
            with open(self.path, "rb") as js:
                # Get axis and button counts via ioctl
                try:
                    import fcntl
                    buf = bytearray(1)
                    fcntl.ioctl(js.fileno(), 0x80016a11, buf)  # JSIOCGAXES
                    self.num_axes = buf[0]
                    fcntl.ioctl(js.fileno(), 0x80016a12, buf)  # JSIOCGBUTTONS
                    self.num_buttons = buf[0]
                    with self.lock:
                        self.controller_model = self._detect_model()
                except:
                    pass

                while self.running:
                    r, _, _ = select.select([js], [], [], 0.1)
                    if not r:
                        continue
                    ev = js.read(8)
                    _, val, typ, num = struct.unpack("IhBB", ev)
                    with self.lock:
                        if typ & self.JS_EVENT_AXIS:
                            self.axes[num] = val / 32767.0
                        elif typ & self.JS_EVENT_BUTTON:
                            self.buttons[num] = val
        except:
            pass

    def state(self):
        with self.lock:
            return dict(self.axes), dict(self.buttons)

    def get_model(self):
        with self.lock:
            return self.controller_model


class DummyJoystickReader:
    """Dummy joystick reader that returns zeros when no controller is connected"""

    def __init__(self):
        self.num_axes = 8
        self.num_buttons = 12
        self.controller_model = "No Controller"

    def start(self):
        pass

    def stop(self):
        pass

    def state(self):
        axes = {i: 0.0 for i in range(self.num_axes)}
        buttons = {i: 0 for i in range(self.num_buttons)}
        return axes, buttons

    def get_model(self):
        return self.controller_model


# ---------------- ROS2 TELEOP NODE ----------------
if ROS2_AVAILABLE:
    class DriveROSNode(Node):
        """ROS2 Node for publishing drive commands via /joy subscription (like RoverTeleopNode)"""

        def __init__(self, joystick_reader=None):
            super().__init__('rover_teleop_node')
            self.reader = joystick_reader  # Optional, for model detection display

            # Subscribe to /joy topic (from ros2 run joy joy_node)
            self.joy_subscriber = self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                10
            )

            # Publishers
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.indicator_pub = self.create_publisher(String, '/indicator', 10)

            # State
            self.dummyNode = None
            self.send_msg = False
            self.controller_model = None
            self.prev_buttons = []

            self.get_logger().info("Drive teleop started - Listening to /joy")

        def detect_controller_model(self, msg):
            """Detect controller model based on button/axis count"""
            if len(msg.buttons) == 11 and len(msg.axes) == 8:
                self.controller_model = "Xbox-360 Controller"
            elif len(msg.buttons) == 13 and len(msg.axes) == 8:
                self.controller_model = "PS4 Controller"
            elif len(msg.buttons) == 12 and len(msg.axes) == 6:
                self.controller_model = "Logitech X-3D Pro"
            else:
                self.controller_model = "Unknown Controller Model"

        def button_pressed(self, msg, idx):
            """Edge detection - returns True only on button press (not hold)"""
            current = msg.buttons[idx] if idx < len(msg.buttons) else 0
            previous = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
            return current == 1 and previous == 0

        def joy_callback(self, msg):
            self.detect_controller_model(msg)

            # Xbox-360 Controller
            if self.controller_model == "Xbox-360 Controller":
                if self.button_pressed(msg, 2) and self.dummyNode is None:
                    self.send_msg = True
                    self.create_dummy_node()
                    self.get_logger().info("Manual Drive")
                elif self.button_pressed(msg, 1) and self.dummyNode is not None:
                    self.send_msg = False
                    self.destroy_dummy_node()
                    self.get_logger().info("Autonomous Drive")

            # PS4 Controller
            elif self.controller_model == "PS4 Controller":
                if self.button_pressed(msg, 3) and self.dummyNode is None:
                    self.send_msg = True
                    self.create_dummy_node()
                    self.get_logger().info("Manual Drive")
                elif self.button_pressed(msg, 1) and self.dummyNode is not None:
                    self.send_msg = False
                    self.destroy_dummy_node()
                    self.get_logger().info("Autonomous Drive")

            # Logitech X-3D Pro
            elif self.controller_model == "Logitech X-3D Pro":
                if self.button_pressed(msg, 1) and self.dummyNode is None:
                    self.send_msg = True
                    self.create_dummy_node()
                    self.get_logger().info("Manual Drive")
                elif self.button_pressed(msg, 2) and self.dummyNode is not None:
                    self.send_msg = False
                    self.destroy_dummy_node()
                    self.get_logger().info("Autonomous Drive")
            else:
                self.get_logger().info("Unknown Controller")

            if self.send_msg:
                self.publish_twist_msg(msg)

            # Store previous button state for edge detection
            self.prev_buttons = list(msg.buttons)

        def create_dummy_node(self):
            """Create dummy node to indicate manual mode is active"""
            self.dummyNode = rclpy.create_node('teleop_is_on')
            indicator_msg = String()
            indicator_msg.data = "Blue -> Manual Mode"
            self.indicator_pub.publish(indicator_msg)

        def destroy_dummy_node(self):
            """Destroy dummy node to indicate autonomous mode"""
            if self.dummyNode is not None:
                self.dummyNode.destroy_node()
                self.dummyNode = None
                indicator_msg = String()
                indicator_msg.data = "RED -> Autonomous Mode"
                self.indicator_pub.publish(indicator_msg)

        def publish_twist_msg(self, joy):
            """Publish twist message based on controller model"""
            twist = Twist()

            if self.controller_model == "Xbox-360 Controller":
                twist.linear.x = (joy.axes[1] / 2) + (joy.axes[4] / 2)
                twist.angular.z = (joy.axes[0] / 2) + (joy.axes[3] / 2)

            elif self.controller_model == "PS4 Controller":
                twist.linear.x = (joy.axes[1] / 2) + (joy.axes[4] / 2)
                twist.angular.z = (joy.axes[0] / 2) + (joy.axes[3] / 2)
                # Remove velocity bound [-0.17, 0.17]
                if joy.axes[0] < 0.17 and joy.axes[0] > -0.17:
                    twist.angular.z -= joy.axes[0] / 2
                if joy.axes[3] < 0.17 and joy.axes[3] > -0.17:
                    twist.angular.z -= joy.axes[3] / 2
                if joy.axes[1] < 0.17 and joy.axes[1] > -0.17:
                    twist.linear.x -= joy.axes[1] / 2
                if joy.axes[4] < 0.17 and joy.axes[4] > -0.17:
                    twist.linear.x -= joy.axes[4] / 2

            elif self.controller_model == "Logitech X-3D Pro":
                if joy.buttons[0]:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                # If 360 rotate is ON and usual traverse is not ON
                elif (joy.axes[2] > 0.17 or joy.axes[2] < -0.17) and not (joy.axes[1] > 0.1 or joy.axes[1] < -0.1):
                    twist.linear.x = 0.0
                    twist.angular.z = joy.axes[2]
                else:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]
                    # Remove angular velocity bound [-0.1, 0.1]
                    if twist.angular.z < 0.1 and twist.angular.z > -0.1:
                        twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)


    class ArmROSNode(Node):
        """ROS2 Node for publishing arm commands via /joy subscription"""

        def __init__(self, joystick_reader=None):
            super().__init__('arm_teleop_node')
            self.reader = joystick_reader  # Optional, for model detection display

            # Subscribe to /joy topic
            self.joy_subscriber = self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                10
            )

            # Publishers for arm control
            self.arm_cmd_pub = self.create_publisher(Twist, '/arm_cmd', 10)
            self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)

            self.get_logger().info("Arm teleop started - Listening to /joy")

        def joy_callback(self, msg):
            # Arm movement via Twist
            twist = Twist()
            twist.linear.x = msg.axes[1] if len(msg.axes) > 1 else 0.0   # Forward/back
            twist.linear.y = msg.axes[0] if len(msg.axes) > 0 else 0.0   # Left/right
            twist.linear.z = msg.axes[4] if len(msg.axes) > 4 else 0.0   # Up/down
            twist.angular.x = msg.axes[3] if len(msg.axes) > 3 else 0.0  # Wrist roll
            twist.angular.y = msg.axes[2] if len(msg.axes) > 2 else 0.0  # Wrist pitch
            self.arm_cmd_pub.publish(twist)

            # Gripper control
            if len(msg.buttons) > 4 and msg.buttons[4]:  # L1/LB
                gripper_msg = String()
                gripper_msg.data = "open"
                self.gripper_pub.publish(gripper_msg)
            elif len(msg.buttons) > 5 and msg.buttons[5]:  # R1/RB
                gripper_msg = String()
                gripper_msg.data = "close"
                self.gripper_pub.publish(gripper_msg)


# ---------------- MENU ----------------
class ControllerMenu:
    def __init__(self):
        self.controllers = []
        self.drive = None
        self.arm = None
        self.drive_on = False
        self.arm_on = False
        self.selection = 0

        # Joystick readers
        self.drive_reader = None
        self.arm_reader = None

        # ROS2 nodes
        self.drive_ros_node = None
        self.arm_ros_node = None
        self.ros_spin_thread = None
        self.ros_initialized = False

        self.menu = [
            "Monitor Controller Input",
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
        """Spin ROS2 nodes in background"""
        while self.ros_initialized:
            try:
                if self.drive_ros_node:
                    rclpy.spin_once(self.drive_ros_node, timeout_sec=0.01)
                if self.arm_ros_node:
                    rclpy.spin_once(self.arm_ros_node, timeout_sec=0.01)
                time.sleep(0.01)
            except:
                break

    def _start_drive_ros(self):
        """Start the Drive ROS2 node (subscribes to /joy topic)"""
        if not ROS2_AVAILABLE:
            return False

        if not self._init_ros():
            return False

        # Optionally start joystick reader for monitor display
        if self.drive and self.drive_reader is None:
            self.drive_reader = JoystickReader(self.drive["path"])
            self.drive_reader.start()

        # Create ROS node - subscribes to /joy, no controller needed here
        try:
            self.drive_ros_node = DriveROSNode(self.drive_reader)
            self._start_ros_spin_thread()
            return True
        except Exception as e:
            return False

    def _stop_drive_ros(self):
        """Stop the Drive ROS2 node"""
        if self.drive_ros_node:
            try:
                self.drive_ros_node.destroy_node()
            except:
                pass
            self.drive_ros_node = None

        if self.drive_reader and not self.drive_on:
            self.drive_reader.stop()
            self.drive_reader = None

    def _start_arm_ros(self):
        """Start the ARM ROS2 node (subscribes to /joy topic)"""
        if not ROS2_AVAILABLE:
            return False

        if not self._init_ros():
            return False

        # Optionally start joystick reader for monitor display
        if self.arm and self.arm_reader is None:
            self.arm_reader = JoystickReader(self.arm["path"])
            self.arm_reader.start()

        # Create ROS node - subscribes to /joy, no controller needed here
        try:
            self.arm_ros_node = ArmROSNode(self.arm_reader)
            self._start_ros_spin_thread()
            return True
        except Exception as e:
            return False

    def _stop_arm_ros(self):
        """Stop the ARM ROS2 node"""
        if self.arm_ros_node:
            try:
                self.arm_ros_node.destroy_node()
            except:
                pass
            self.arm_ros_node = None

        if self.arm_reader and not self.arm_on:
            self.arm_reader.stop()
            self.arm_reader = None

    # -------- REAL-TIME DETECT + AUTO ASSIGN --------
    def refresh_controllers(self):
        self.controllers = find_joysticks()

        if not self.controllers:
            self.drive = None
            self.arm = None
            return

        if len(self.controllers) == 1:
            self.drive = self.controllers[0]
            self.arm = self.controllers[0]
        else:
            if self.drive not in self.controllers:
                self.drive = self.controllers[0]
            if self.arm not in self.controllers:
                self.arm = self.controllers[1]

    # ---------------- DRAW ----------------
    def draw(self, stdscr):
        stdscr.clear()
        h, w = stdscr.getmaxyx()

        self.refresh_controllers()

        stdscr.addstr(1, 2, "ROS CONTROLLER CONFIGURATION", curses.A_BOLD | curses.color_pair(4))

        # ROS2 availability indicator
        if ROS2_AVAILABLE:
            stdscr.addstr(1, 35, "[ROS2 OK]", curses.color_pair(2))
        else:
            stdscr.addstr(1, 35, "[ROS2 N/A]", curses.color_pair(1))

        # DRIVE
        y = 3
        stdscr.addstr(y, 2, "DRIVE", curses.A_BOLD | curses.color_pair(3))
        if self.drive:
            name_display = self.drive["name"][:w-10] if len(self.drive["name"]) > w-10 else self.drive["name"]
            stdscr.addstr(y + 1, 4, name_display, curses.color_pair(2))
            # Show detected model
            if self.drive_reader:
                model = self.drive_reader.get_model()
                stdscr.addstr(y + 2, 4, f"Model: {model}", curses.color_pair(4))
        else:
            stdscr.addstr(y + 1, 4, "Not detected", curses.color_pair(1))

        ros_status = "● PUBLISHING" if self.drive_on and self.drive_ros_node else ("○ INACTIVE" if not self.drive_on else "○ NO ROS2")
        color = curses.color_pair(2) if (self.drive_on and self.drive_ros_node) else curses.color_pair(1)
        stdscr.addstr(y + 3, 4, f"ROS: {ros_status}", color)

        # ARM
        y += 5
        stdscr.addstr(y, 2, "ARM", curses.A_BOLD | curses.color_pair(3))
        if self.arm:
            name_display = self.arm["name"][:w-10] if len(self.arm["name"]) > w-10 else self.arm["name"]
            stdscr.addstr(y + 1, 4, name_display, curses.color_pair(2))
            if self.arm_reader:
                model = self.arm_reader.get_model()
                stdscr.addstr(y + 2, 4, f"Model: {model}", curses.color_pair(4))
        else:
            stdscr.addstr(y + 1, 4, "Not detected", curses.color_pair(1))

        ros_status = "● PUBLISHING" if self.arm_on and self.arm_ros_node else ("○ INACTIVE" if not self.arm_on else "○ NO ROS2")
        color = curses.color_pair(2) if (self.arm_on and self.arm_ros_node) else curses.color_pair(1)
        stdscr.addstr(y + 3, 4, f"ROS: {ros_status}", color)

        # MENU
        y += 5
        for i, item in enumerate(self.menu):
            if i == self.selection:
                stdscr.attron(curses.A_REVERSE)
            stdscr.addstr(y + i, 4, item)
            if i == self.selection:
                stdscr.attroff(curses.A_REVERSE)

        stdscr.addstr(
            h - 1, 2,
            "↑↓ Navigate  ENTER Select  Q Quit",
            curses.A_REVERSE
        )
        stdscr.refresh()

    # ---------------- POPUP ----------------
    def popup_select(self, stdscr, title, assign):
        if not self.controllers:
            return

        sel = 0
        while True:
            stdscr.clear()
            stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))

            for i, d in enumerate(self.controllers):
                line = f"{d['name']} ({d['path']})"
                if i == sel:
                    stdscr.attron(curses.A_REVERSE)
                stdscr.addstr(4 + i, 6, line)
                if i == sel:
                    stdscr.attroff(curses.A_REVERSE)

            stdscr.addstr(6 + len(self.controllers), 4, "ENTER Select   B Back")
            stdscr.refresh()

            key = stdscr.getch()
            if key == curses.KEY_UP:
                sel = (sel - 1) % len(self.controllers)
            elif key == curses.KEY_DOWN:
                sel = (sel + 1) % len(self.controllers)
            elif key in (10, 13):
                assign(self.controllers[sel])
                break
            elif key in (ord('b'), ord('B')):
                break

    # ---------------- MONITOR ----------------
    def monitor(self, stdscr):
        readers = []
        temp_readers = []  # Track temporary readers to stop later

        if self.drive:
            if self.drive_reader:
                readers.append(("DRIVE", self.drive_reader))
            else:
                r = JoystickReader(self.drive["path"])
                r.start()
                readers.append(("DRIVE", r))
                temp_readers.append(r)
        else:
            # Use dummy reader if no drive controller selected
            r = DummyJoystickReader()
            readers.append(("DRIVE (No Controller)", r))

        if self.arm and self.arm != self.drive:
            if self.arm_reader:
                readers.append(("ARM", self.arm_reader))
            else:
                r = JoystickReader(self.arm["path"])
                r.start()
                readers.append(("ARM", r))
                temp_readers.append(r)
        elif not self.arm:
            # Use dummy reader if no arm controller selected
            r = DummyJoystickReader()
            readers.append(("ARM (No Controller)", r))

        stdscr.nodelay(True)
        try:
            while True:
                stdscr.clear()
                h, w = stdscr.getmaxyx()
                stdscr.addstr(1, 2, "CONTROLLER MONITOR", curses.A_BOLD | curses.color_pair(4))
                y = 3

                for name, r in readers:
                    axes, btns = r.state()
                    model = r.get_model()

                    stdscr.addstr(y, 2, f"{name}: {model}", curses.A_BOLD | curses.color_pair(3))
                    y += 1

                    # Display axes with visual bars
                    stdscr.addstr(y, 2, "Axes:", curses.A_UNDERLINE)
                    y += 1
                    for i in range(min(8, r.num_axes if r.num_axes else 8)):
                        val = axes.get(i, 0.0)
                        bar_w = 15
                        bar_pos = int((val + 1) / 2 * bar_w)
                        bar = "─" * bar_pos + "█" + "─" * (bar_w - bar_pos - 1)
                        if y < h - 4:
                            stdscr.addstr(y, 4, f"A{i}: [{bar}] {val:+.2f}")
                            y += 1

                    # Display buttons
                    stdscr.addstr(y, 2, "Buttons:", curses.A_UNDERLINE)
                    y += 1
                    btn_line = ""
                    for i in range(min(16, r.num_buttons if r.num_buttons else 16)):
                        val = btns.get(i, 0)
                        if val:
                            btn_line += f"[B{i}] "
                        else:
                            btn_line += f" B{i}  "
                    if y < h - 3:
                        stdscr.addstr(y, 4, btn_line[:w-6], curses.color_pair(2))
                    y += 2

                stdscr.addstr(h - 2, 2, "Press Q to go back", curses.A_REVERSE)
                stdscr.refresh()

                key = stdscr.getch()
                if key in (ord('q'), ord('Q')):
                    break
                time.sleep(0.05)
        finally:
            stdscr.nodelay(False)
            # Only stop readers we created locally
            for r in temp_readers:
                r.stop()

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
                        self.monitor(stdscr)
                    elif self.selection == 1:
                        self.popup_select(stdscr, "Select DRIVE Controller", lambda d: setattr(self, "drive", d))
                    elif self.selection == 2:
                        self.popup_select(stdscr, "Select ARM Controller", lambda d: setattr(self, "arm", d))
                    elif self.selection == 3:
                        # Toggle DRIVE ROS Node (subscribes to /joy, no controller required)
                        if not self.drive_on:
                            if self._start_drive_ros():
                                self.drive_on = True
                            else:
                                self._show_message(stdscr, "Error", "Failed to start ROS node. Check ROS2 installation.")
                        else:
                            self._stop_drive_ros()
                            self.drive_on = False
                    elif self.selection == 4:
                        # Toggle ARM ROS Node (subscribes to /joy, no controller required)
                        if not self.arm_on:
                            if self._start_arm_ros():
                                self.arm_on = True
                            else:
                                self._show_message(stdscr, "Error", "Failed to start ROS node. Check ROS2 installation.")
                        else:
                            self._stop_arm_ros()
                            self.arm_on = False
                    elif self.selection == 5:
                        break
                elif key in (ord('q'), ord('Q')):
                    break
        finally:
            # Cleanup
            self._stop_drive_ros()
            self._stop_arm_ros()
            if self.drive_reader:
                self.drive_reader.stop()
            if self.arm_reader:
                self.arm_reader.stop()
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
