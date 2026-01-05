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
        if event.type == 3: # EV_ABS
            if event.code < len(self.joy_msg.axes):
                self.joy_msg.axes[event.code] = float(event.value) / 32767.0
        elif event.type == 1: # EV_KEY
            if event.code < len(self.joy_msg.buttons):
                self.joy_msg.buttons[event.code] = 1 if event.value > 0 else 0

    def stop(self):
        self.running = False