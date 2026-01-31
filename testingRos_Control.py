#!/usr/bin/env python3
"""
ROS Controller Configuration Menu
- Manual controller selection with MAC address identification
- Separate DRIVE and ARM control systems (no conflicts)
- Modular button/axis mappings for easy customization
- Colorful & clean UI

Simplified version - Controller selection only
"""

import curses
from curses import wrapper
import os
import glob


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


# ---------------- MENU ----------------
class ControllerMenu:
    def __init__(self):
        self.controllers = []
        self.drive = None
        self.arm = None
        self.drive_state = False  # False = "Hello World", True = "Bye World"
        self.arm_state = False    # False = "Hello World", True = "Bye World"
        self.selection = 0

        self.menu = [
            "Select DRIVE Controller",
            "Select ARM Controller",
            "Toggle DRIVE State",
            "Toggle ARM State",
            "Exit"
        ]

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
        self.refresh_controllers()

        stdscr.addstr(1, 2, "ROS CONTROLLER CONFIGURATION", curses.A_BOLD | curses.color_pair(4))

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
        
        # Show state
        state_text = "Bye World" if self.drive_state else "Hello World"
        try:
            stdscr.addstr(y + 4, 4, f"State: {state_text}", curses.color_pair(2 if self.drive_state else 4))
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

        # Show state
        state_text = "Bye World" if self.arm_state else "Hello World"
        try:
            stdscr.addstr(y + 4, 4, f"State: {state_text}", curses.color_pair(2 if self.arm_state else 4))
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
                    # Toggle DRIVE State - print to terminal
                    if not self.drive_state:
                        print("Hello World")
                        self.drive_state = True
                    else:
                        print("Bye World")
                        self.drive_state = False
                elif self.selection == 3:
                    # Toggle ARM State - print to terminal
                    if not self.arm_state:
                        print("Hello World")
                        self.arm_state = True
                    else:
                        print("Bye World")
                        self.arm_state = False
                elif self.selection == 4:
                    break
            elif key in (ord('r'), ord('R')):
                # Manual refresh
                self.refresh_controllers()
            elif key in (ord('q'), ord('Q')):
                break


def main(stdscr):
    ControllerMenu().run(stdscr)


if __name__ == "__main__":
    wrapper(main)