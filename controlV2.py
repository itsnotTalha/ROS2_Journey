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
import json

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

# -------------------- CONTROLLER OWNER NAMES --------------------
# Map MAC addresses to custom owner names for fun!
CONTROLLER_OWNERS = {
    "50:ee:32:04:32:53": "Chagol Chor",
    "84:30:95:41:0e:74": "Goru Chor",
}


ASSIGNMENTS_FILE = "/tmp/ros_controller_assignments.json"


def load_assignments():
    try:
        with open(ASSIGNMENTS_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


def save_assignments(assignments):
    try:
        with open(ASSIGNMENTS_FILE, "w") as f:
            json.dump(assignments, f)
    except Exception:
        pass


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
        """
        def __init__(self):
            super().__init__("conditional_teleop")
            # Publishers are created lazily when DRIVE/ARM is enabled so
            # /buswala and /aram only appear in the graph when active.
            self.drive_cmd_publisher = None
            self.arm_cmd_publisher = None

            self.drive_sub = None
            self.arm_sub = None

            self.drive_dummy_node = None
            self.safety_timer = None

            self._last_drive_model = None
            self._last_arm_model = None

        # ================== DRIVE / ARM ENABLE/DISABLE ==================

        def enable_drive(self):
            if self.drive_sub is None:
                self.drive_sub = self.create_subscription(
                    Joy,
                    "/joy/drive",
                    self._drive_joy_callback,
                    10,
                )
            if self.drive_cmd_publisher is None:
                self.drive_cmd_publisher = self.create_publisher(Twist, "/buswala", 10)
            self.drive_dummy_node = object()

        def disable_drive(self):
            if self.drive_sub is not None:
                self.destroy_subscription(self.drive_sub)
                self.drive_sub = None
            self.drive_dummy_node = None

            # Publish zero once then remove publisher so /buswala
            # disappears from the topic list when inactive.
            if self.drive_cmd_publisher is not None:
                twist = Twist()
                self.drive_cmd_publisher.publish(twist)
                self.destroy_publisher(self.drive_cmd_publisher)
                self.drive_cmd_publisher = None

        def enable_arm(self):
            if self.arm_sub is None:
                self.arm_sub = self.create_subscription(
                    Joy,
                    "/joy/arm",
                    self._arm_joy_callback,
                    10,
                )
            if self.arm_cmd_publisher is None:
                self.arm_cmd_publisher = self.create_publisher(Twist, "/aram", 10)

        def disable_arm(self):
            if self.arm_sub is not None:
                self.destroy_subscription(self.arm_sub)
                self.arm_sub = None

            # Publish zero once then remove publisher so /aram
            # disappears from the topic list when inactive.
            if self.arm_cmd_publisher is not None:
                twist = Twist()
                self.arm_cmd_publisher.publish(twist)
                self.destroy_publisher(self.arm_cmd_publisher)
                self.arm_cmd_publisher = None

        # ================== INTERNAL HELPERS ==================

        def _detect_controller_model(self, joy: Joy) -> str:
            buttons = len(joy.buttons)
            axes = len(joy.axes)
            for name, profile in CONTROLLER_PROFILES.items():
                if profile["buttons"] == buttons and profile["axes"] == axes:
                    return name
            return "Unknown"

        def _drive_joy_callback(self, joy: Joy):
            model = self._detect_controller_model(joy)
            self._last_drive_model = model
            twist = Twist()

            if model == "Xbox-360 Controller":
                twist.linear.x = (joy.axes[1] / 2.0) + (joy.axes[4] / 2.0)
                twist.angular.z = (joy.axes[0] / 2.0) + (joy.axes[3] / 2.0)

            elif model == "PS4 Controller":
                twist.linear.x = (joy.axes[1] / 2.0) + (joy.axes[4] / 2.0)
                twist.angular.z = (joy.axes[0] / 2.0) + (joy.axes[3] / 2.0)

                if -0.17 < joy.axes[0] < 0.17:
                    twist.angular.z -= joy.axes[0] / 2.0
                if -0.17 < joy.axes[3] < 0.17:
                    twist.angular.z -= joy.axes[3] / 2.0
                if -0.17 < joy.axes[1] < 0.17:
                    twist.linear.x -= joy.axes[1] / 2.0
                if -0.17 < joy.axes[4] < 0.17:
                    twist.linear.x -= joy.axes[4] / 2.0

            elif model == "Logitech X-3D Pro":
                if len(joy.buttons) > 0 and joy.buttons[0]:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif (len(joy.axes) > 2 and (joy.axes[2] > 0.17 or joy.axes[2] < -0.17)) and not (
                    len(joy.axes) > 1 and (joy.axes[1] > 0.1 or joy.axes[1] < -0.1)
                ):
                    twist.linear.x = 0.0
                    twist.angular.z = joy.axes[2]
                else:
                    if len(joy.axes) > 1:
                        twist.linear.x = joy.axes[1]
                    if len(joy.axes) > 0:
                        twist.angular.z = joy.axes[0]
                    if -0.1 < twist.angular.z < 0.1:
                        twist.angular.z = 0.0

            if self.drive_cmd_publisher is not None:
                self.drive_cmd_publisher.publish(twist)

        def _arm_joy_callback(self, joy: Joy):
            model = self._detect_controller_model(joy)
            self._last_arm_model = model

            twist = Twist()

            if model in ("Xbox-360 Controller", "PS4 Controller"):
                if len(joy.axes) >= 2:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]
            elif model == "Logitech X-3D Pro":
                if len(joy.axes) >= 2:
                    twist.linear.x = joy.axes[1]
                    twist.angular.z = joy.axes[0]

            if self.arm_cmd_publisher is not None:
                self.arm_cmd_publisher.publish(twist)

        # ================== CLEANUP / MODE QUERY ==================

        def cleanup(self):
            self.disable_drive()
            self.disable_arm()
            if self.safety_timer:
                self.safety_timer.cancel()
                self.destroy_timer(self.safety_timer)

        def is_manual_mode(self):
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

    # -------- UNIVERSAL TOPIC SCANNER --------

    def _scan_topic_exists(self, topic_name: str) -> bool:
        """
        Universal topic scanner - checks if a topic exists in the ROS graph.
        This method scans all available topics in ROS2 to determine if the
        specified topic is currently active/publishing.
        
        Args:
            topic_name: The topic to check (e.g., '/buswala' or '/aram')
        
        Returns:
            bool: True if topic exists in the graph, False otherwise
        """
        if not ROS2_AVAILABLE or not self.ros_initialized:
            return False
        
        try:
            # Create a temporary node for topic discovery if teleop_node doesn't exist
            if self.teleop_node:
                node_for_query = self.teleop_node
            else:
                # If no teleop node exists yet, we need to check the graph differently
                # In this case, we'll try to initialize ROS and create a temporary minimal node
                if not self._init_ros():
                    return False
                # Without a node, we can't query topics, so return False
                return False
            
            # Get all available topics in the ROS graph
            topics = node_for_query.get_topic_names_and_types()
            
            # Check if our target topic exists
            for name, _types in topics:
                if name == topic_name:
                    return True
            
            return False
        except Exception:
            return False

    # -------- CONTROLLER REFRESH (NO AUTO-ASSIGN) --------
    def refresh_controllers(self):
        """Refresh controller list but do NOT auto-assign.
        User must manually select controllers.
        """
        self.controllers = find_joysticks()

        # Restore from global assignments if possible
        assignments = load_assignments()
        if not self.drive and assignments.get("drive"):
            for ctrl in self.controllers:
                if ctrl["mac"] == assignments["drive"]:
                    self.drive = ctrl
                    break
        if not self.arm and assignments.get("arm"):
            for ctrl in self.controllers:
                if ctrl["mac"] == assignments["arm"]:
                    self.arm = ctrl
                    break

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
        
        # Drive ROS status - scan topics universally
        d_ros_active = self._scan_topic_exists("/buswala")
        d_ros = "* PUBLISHING" if d_ros_active else "o INACTIVE"
        try:
            stdscr.addstr(y + 4, 4, f"ROS: {d_ros}", curses.color_pair(2 if d_ros_active else 1))
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

        # ARM ROS status - scan topics universally
        a_ros_active = self._scan_topic_exists("/aram")
        a_ros = "* PUBLISHING" if a_ros_active else "o INACTIVE"
        try:
            stdscr.addstr(y + 4, 4, f"ROS: {a_ros}", curses.color_pair(2 if a_ros_active else 1))
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
    def popup_select(self, stdscr, title, assign, role, exclude_controller=None):
        """Popup for manual controller selection with MAC addresses.
        exclude_controller: controller dict that should be shown as unavailable (already in use)
        role: "drive" or "arm" for global assignment tracking
        """
        sel = 0
        while True:
            # Refresh controller list
            self.controllers = find_joysticks()

            # Load global assignments
            assignments = load_assignments()
            current_ctrl = getattr(self, role, None)
            current_mac = current_ctrl.get("mac") if current_ctrl else None
            global_in_use_macs = set()
            for mac in assignments.values():
                if mac and mac != current_mac:
                    global_in_use_macs.add(mac)

            # Find available controllers (not already used by the other system
            # and not globally in use)
            available_indices = []
            for i, d in enumerate(self.controllers):
                mac = d.get("mac")
                if exclude_controller and mac == exclude_controller.get("mac"):
                    continue  # This controller is in use by the other local role
                if mac in global_in_use_macs:
                    continue  # This controller is globally in use
                available_indices.append(i)
            
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            stdscr.addstr(2, 4, title, curses.A_BOLD | curses.color_pair(4))

            if not self.controllers:
                stdscr.addstr(4, 6, "No controllers found.", curses.color_pair(1))
                stdscr.addstr(5, 6, "Connect a controller and press R to refresh.", curses.color_pair(4))
            else:
                for i, d in enumerate(self.controllers):
                    mac = d.get("mac")
                    # Check if this controller is already in use
                    is_unavailable = (
                        (exclude_controller and mac == exclude_controller.get("mac"))
                        or (mac in global_in_use_macs)
                    )
                    
                    # Show owner name if available, otherwise show controller name
                    owner = d.get("owner")
                    if owner:
                        line1 = f"[P] {owner} ({d['name']})"
                    else:
                        line1 = f"{d['name']}"
                    
                    # Add unavailable indicator
                    if is_unavailable:
                        line1 = f"[X] {line1} [IN USE]"

                    mac_display = mac[:30] if mac else "N/A"
                    line2 = f"  Path: {d['path']}  MAC: {mac_display}"
                    
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
                chosen = self.controllers[sel]
                assign(chosen)

                # Save global assignment for this role
                assignments = load_assignments()
                assignments[role] = chosen.get("mac")
                save_assignments(assignments)
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
                        self.popup_select(
                            stdscr,
                            "Select DRIVE Controller",
                            lambda d: setattr(self, "drive", d),
                            "drive",
                            exclude_controller=self.arm,
                        )
                    elif self.selection == 1:
                        # Select ARM - exclude DRIVE controller if selected
                        self.popup_select(
                            stdscr,
                            "Select ARM Controller",
                            lambda d: setattr(self, "arm", d),
                            "arm",
                            exclude_controller=self.drive,
                        )
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

            # After leaving the loop, remove our assignments from the global file
            assignments = load_assignments()
            if self.drive:
                drive_mac = self.drive.get("mac")
                if assignments.get("drive") == drive_mac:
                    assignments.pop("drive", None)
            if self.arm:
                arm_mac = self.arm.get("mac")
                if assignments.get("arm") == arm_mac:
                    assignments.pop("arm", None)
            save_assignments(assignments)

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