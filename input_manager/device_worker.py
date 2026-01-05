import threading
import time
import rclpy
from rclpy.clock import Clock  # Correct way to import Clock
from evdev import InputDevice, list_devices, ecodes
from sensor_msgs.msg import Joy

class DeviceWorker(threading.Thread):
    def __init__(self, config, publisher):
        super().__init__(daemon=True)
        self.config = config
        self.publisher = publisher
        self.running = True
        self.device = None
        self.is_connected = False
        
        # Initialize a standalone ROS2 clock
        self.ros_clock = Clock()
        
        # Initialize Joy message
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12

    def _find_device_path(self):
        for path in list_devices():
            try:
                dev = InputDevice(path)
                vid_pid = f"{dev.info.vendor:04x}:{dev.info.product:04x}"
                if vid_pid == self.config['id']:
                    return dev
            except Exception:
                continue
        return None

    def process_event(self, event):
        """Maps raw hardware events to the Joy message."""
        # Using ecodes.by_name or direct integers solves the linter error
        # EV_ABS is 3, EV_KEY is 1
        
        # Handle Axes
        if event.type == 3: # Equivalent to ecodes.EV_ABS
            if event.code < len(self.joy_msg.axes):
                # Standard normalization for 16-bit joystick input
                self.joy_msg.axes[event.code] = float(event.value) / 32767.0
        
        # Handle Buttons
        elif event.type == 1: # Equivalent to ecodes.EV_KEY
            if event.code < len(self.joy_msg.buttons):
                self.joy_msg.buttons[event.code] = 1 if event.value > 0 else 0

    def run(self):
        while self.running:
            if not self.is_connected or self.device is None:
                self.device = self._find_device_path()
                if self.device:
                    self.is_connected = True
                    print(f"[SUCCESS] Connected to {self.config['name']}")
                else:
                    time.sleep(2.0)
                    continue

            try:
                # Use a local reference to avoid None-type race conditions
                active_dev = self.device
                if active_dev is not None:
                    for event in active_dev.read_loop():
                        if not self.running: 
                            break
                        
                        self.process_event(event)
                        
                        # Fix for: "clock" is not a known attribute of module "rclpy"
                        self.joy_msg.header.stamp = self.ros_clock.now().to_msg()
                        self.joy_msg.header.frame_id = self.config['name']
                        self.publisher.publish(self.joy_msg)
                else:
                    self.is_connected = False

            except (OSError, AttributeError, RuntimeError) as e:
                print(f"[DISCONNECT] {self.config['name']} lost: {e}")
                self.device = None
                self.is_connected = False

    def stop(self):
        self.running = False