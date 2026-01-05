import threading
import time
import rclpy
from rclpy.clock import Clock
from evdev import InputDevice, list_devices
from sensor_msgs.msg import Joy

class DeviceWorker(threading.Thread):
    def __init__(self, config, publisher, node):
        super().__init__(daemon=True)
        self.config = config
        self.publisher = publisher
        self.node = node
        self.running = True
        self.device = None
        self.is_connected = False
        self.ros_clock = Clock()
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12

                # Button mapping: event.code -> joy_msg.buttons index
        # Common Xbox/PS controller button codes
        self.button_map = {
            304: 0,  # BTN_SOUTH (A/X)
            305: 1,  # BTN_EAST (B/Circle)
            306: 2,  # BTN_NORTH (X/Square)
            307: 3,  # BTN_WEST (Y/Triangle)
            308: 4,  # BTN_TL (LB/L1)
            309: 5,  # BTN_TR (RB/R1)
            310: 6,  # BTN_SELECT (Back/Share)
            311: 7,  # BTN_START (Start/Options)
            312: 8,  # BTN_MODE (Xbox/PS button)
            313: 9,  # BTN_THUMBL (Left stick press)
            314: 10, # BTN_THUMBR (Right stick press)
        }
        
        # Axis mapping (if needed, though axes usually start at 0)
        self.axis_map = {
            0: 0,  # Left stick X
            1: 1,  # Left stick Y
            2: 2,  # Left trigger
            3: 3,  # Right stick X
            4: 4,  # Right stick Y
            5: 5,  # Right trigger
            16: 6, # D-pad X
            17: 7, # D-pad Y
        }

    def _reset_and_publish_zeros(self):
        """Safety: Sets all inputs to 0.0 and publishes once."""
        self.joy_msg.axes = [0.0] * len(self.joy_msg.axes)
        self.joy_msg.buttons = [0] * len(self.joy_msg.buttons)
        self.joy_msg.header.stamp = self.ros_clock.now().to_msg()
        self.publisher.publish(self.joy_msg)
        self.node.get_logger().warn(f"[{self.config['name']}] Connection lost: Output zeroed.")

    def run(self):
        while self.running:
            if not self.is_connected or self.device is None:
                self.device = self._find_device_path()
                if self.device:
                    self.is_connected = True
                else:
                    time.sleep(1.0)
                    continue

            try:
                for event in self.device.read_loop():
                    if not self.running: break
                    self.process_event(event)
                    self.joy_msg.header.stamp = self.ros_clock.now().to_msg()
                    self.publisher.publish(self.joy_msg)
            except (OSError, AttributeError, RuntimeError):
                self.is_connected = False
                self.device = None
                self._reset_and_publish_zeros()

    def _find_device_path(self):
        for path in list_devices():
            try:
                dev = InputDevice(path)
                if f"{dev.info.vendor:04x}:{dev.info.product:04x}" == self.config['id']:
                    return dev
            except: continue
        return None

    def process_event(self, event):
        if event.type == 3:  # EV_ABS
            self.node.get_logger().info(f"Axis event - code: {event.code}, value: {event.value}")
            if event.code in self.axis_map:
                idx = self.axis_map[event.code]
                self.joy_msg.axes[idx] = float(event.value) / 32767.0
        elif event.type == 1:  # EV_KEY
            self.node.get_logger().info(f"Button event - code: {event.code}, value: {event.value}")
            if event.code in self.button_map:
                idx = self.button_map[event.code]
                self.joy_msg.buttons[idx] = 1 if event.value > 0 else 0

    def stop(self):
        self.running = False