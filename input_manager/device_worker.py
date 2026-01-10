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

        # Initialize Joy message (12 axises and 24 buttons tu support all possible inputs)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 12
        self.joy_msg.buttons = [0] * 24

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
            # 3 = EV_ABS (Axes)
            if event.type == 3:
                if event.code in self.axis_map:
                    idx = self.axis_map[event.code]
                    
                    # 1. Normalize hardware value (-32767 to 32767) to (-1.0 to 1.0)
                    val = float(event.value) / 32767.0
                    
                    # 2. Apply Sanitization/Deadzone
                    if self.config.get('sanitize', False):
                        deadzone = self.config.get('deadzone', 0.0)
                        if abs(val) < deadzone:
                            val = 0.0
                    
                    self.joy_msg.axes[idx] = val

            # 1 = EV_KEY (Buttons)
            elif event.type == 1:
                if event.code in self.btn_map:
                    idx = self.btn_map[event.code]
                    self.joy_msg.buttons[idx] = 1 if event.value > 0 else 0

    def run(self):
        last_joy_time = 0
        last_wd_time = 0
        previously_connected = False

        while self.running:
            current_time = time.time()

            if not self.is_connected or self.device is None:
                self.device = self._find_device()
                if self.device:
                    self.is_connected = True
                    previously_connected = True
                    self.node.get_logger().info(f"[{self.config['name']}] Connected.")
                else:
                    if previously_connected:
                        self.node.get_logger().warn(f"[{self.config['name']}] Connection Lost.")
                        previously_connected = False
                    self.is_connected = False
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