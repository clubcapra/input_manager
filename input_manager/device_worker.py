import threading
import time
import rclpy
from rclpy.clock import Clock
from evdev import InputDevice, list_devices
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class DeviceWorker(threading.Thread):
    def __init__(self, config, node):
        super().__init__(daemon=True)
        self.config = config
        self.node = node
        self.running = True
        self.device = None
        self.is_connected = False
        self.ros_clock = Clock()
        
        # Setup publishers
        alias = self.config.get('alias', 'unknown')
        self.joy_pub = self.node.create_publisher(Joy, f'/joy_input/{alias}', 10)
        self.wd_pub = self.node.create_publisher(Bool, f'/input_watchdog/{alias}', 10)

        # Initialize Joy message (8 axes, 16 buttons as per Steam Deck spec)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 16

        # Mappings from YAML (keys are indices, values are evdev codes)
        # We invert them for faster lookup: evdev_code -> joy_index
        mapping = self.config.get('mapping', {})
        self.btn_map = {int(v): int(k) for k, v in mapping.get('buttons', {}).items()}
        self.axis_map = {int(v): int(k) for k, v in mapping.get('axes', {}).items()}

    def _find_device(self):
        for path in list_devices():
            try:
                dev = InputDevice(path)
                if f"{dev.info.vendor:04x}:{dev.info.product:04x}" == self.config['id']:
                    return dev
            except: continue
        return None

    def process_event(self, event):
        if event.type == 3:  # EV_ABS
            if event.code in self.axis_map:
                idx = self.axis_map[event.code]
                # Standard 16-bit normalization
                self.joy_msg.axes[idx] = float(event.value) / 32767.0
        elif event.type == 1:  # EV_KEY
            if event.code in self.btn_map:
                idx = self.btn_map[event.code]
                self.joy_msg.buttons[idx] = 1 if event.value > 0 else 0

    def run(self):
        last_joy_time = 0
        last_wd_time = 0

        while self.running:
            current_time = time.time()

            # 1. Hardware Connection Management
            if not self.is_connected or self.device is None:
                self.device = self._find_device()
                if self.device:
                    self.is_connected = True
                    self.node.get_logger().info(f"[{self.config['name']}] Connected.")
                else:
                    self.is_connected = False
                    # Force zeros if disconnected
                    self.joy_msg.axes = [0.0] * 8
                    self.joy_msg.buttons = [0] * 16

            # 2. Read pending hardware events (Non-blocking)
            if self.is_connected and self.device:
                try:
                    while True:
                        event = self.device.read_one()
                        if event is None: break
                        self.process_event(event)
                except (OSError, RuntimeError):
                    self.is_connected = False
                    self.device = None

            # 3. 10Hz Joy Heartbeat (100ms)
            if (current_time - last_joy_time) >= 0.1:
                self.joy_msg.header.stamp = self.ros_clock.now().to_msg()
                # Frame ID as status string for extra safety
                self.joy_msg.header.frame_id = "OK" if self.is_connected else "D/C"
                self.joy_pub.publish(self.joy_msg)
                last_joy_time = current_time

            # 4. 5Hz Watchdog (200ms)
            if (current_time - last_wd_time) >= 0.2:
                wd_msg = Bool()
                wd_msg.data = not self.is_connected # 0 if connected, 1 if disconnected
                self.wd_pub.publish(wd_msg)
                last_wd_time = current_time

            time.sleep(0.01) # High frequency spin

    def stop(self):
        self.running = False