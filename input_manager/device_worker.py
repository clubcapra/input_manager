import threading
import time
import os
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

        alias = self.config.get('alias', 'unknown')
        self.joy_pub = self.node.create_publisher(Joy, f'/joy_input/{alias}', 10)
        self.wd_pub  = self.node.create_publisher(Bool, f'/input_watchdog/{alias}', 10)

        self.joy_msg = Joy()
        self.joy_msg.axes    = [0.0] * 12
        self.joy_msg.buttons = [0]   * 24

        mapping = self.config.get('mapping', {})

        # evdev_code -> joy_index  (inverted from YAML for fast lookup)
        self.btn_map  = {int(v): int(k) for k, v in mapping.get('buttons', {}).items()}
        self.axis_map = {int(v): int(k) for k, v in mapping.get('axes',    {}).items()}

        # Per-axis raw integer range, for axes that don't use the ±32767 standard.
        # Format: evdev_code -> [raw_min, raw_max]
        # These are the actual integers evdev sends for that axis.
        # D-PAD example: sends -1 / 0 / 1  →  declare as [-1, 1]
        # Omit an axis here and it gets divided by 32767 (normal stick behaviour).
        axis_ranges_cfg = mapping.get('axis_ranges', {})
        self.axis_ranges = {int(k): v for k, v in axis_ranges_cfg.items()}

        # Mirror an axis value to two button slots.
        # Threshold is in the final [-1, 1] space.
        # neg_button fires when val < -threshold, pos_button when val > +threshold.
        self.axes_as_buttons = [
            {
                'axis_code':  int(e['axis_code']),
                'neg_button': int(e['neg_button']),
                'pos_button': int(e['pos_button']),
                'threshold':  float(e.get('threshold', 0.5)),
            }
            for e in mapping.get('axes_as_buttons', [])
        ]
        print("axis_ranges loaded:", self.axis_ranges)

    # ------------------------------------------------------------------

    def _find_device(self):
        udev_path = self.config.get('udev_path')
        if udev_path and os.path.exists(udev_path):
            try:
                return InputDevice(udev_path)
            except Exception as e:
                self.node.get_logger().error(
                    f"[{self.config['name']}] udev path exists but failed: {e}")

        target_id = self.config.get('id')
        for path in list_devices():
            try:
                dev = InputDevice(path)
                if f"{dev.info.vendor:04x}:{dev.info.product:04x}" == target_id:
                    if 3 in dev.capabilities():  # EV_ABS = real controller, not keyboard clone
                        self.node.get_logger().info(
                            f"[{self.config['name']}] Found via ID scan on {path}")
                        return dev
            except Exception:
                continue
        return None

    def _normalize(self, evdev_code: int, raw: int) -> float:
        """Map a raw evdev integer to [-1.0, 1.0].

        Sticks:  evdev sends -32768…32767  →  divide by 32767  (default)
        D-PAD:   evdev sends -1 / 0 / 1   →  declare axis_ranges: {16: [-1,1], 17: [-1,1]}
        """
        if evdev_code in self.axis_ranges:
            lo, hi = self.axis_ranges[evdev_code]
            span = hi - lo
            if span == 0:
                return 0.0
            return max(-1.0, min(1.0, 2.0 * (raw - lo) / span - 1.0))
        return max(-1.0, min(1.0, raw / 32767.0))

    def process_event(self, event):
        if event.type == 3:  # EV_ABS — axes
            if event.code not in self.axis_map:
                return

            val = self._normalize(event.code, event.value)

            if self.config.get('sanitize', False):
                if abs(val) < self.config.get('deadzone', 0.0):
                    val = 0.0

            self.joy_msg.axes[self.axis_map[event.code]] = val

            for e in self.axes_as_buttons:
                if e['axis_code'] != event.code:
                    continue
                t = e['threshold']
                self.joy_msg.buttons[e['neg_button']] = 1 if val < -t else 0
                self.joy_msg.buttons[e['pos_button']] = 1 if val >  t else 0

        elif event.type == 1:  # EV_KEY — buttons
            if event.code in self.btn_map:
                self.joy_msg.buttons[self.btn_map[event.code]] = 1 if event.value > 0 else 0

    # ------------------------------------------------------------------

    def run(self):
        last_joy_time = 0.0
        last_wd_time  = 0.0
        previously_connected = False

        while self.running:
            now = time.time()

            if not self.is_connected:
                self.device = self._find_device()
                if self.device:
                    self.is_connected = True
                    previously_connected = True
                    self.node.get_logger().info(f"[{self.config['name']}] Connected.")
                else:
                    if previously_connected:
                        self.node.get_logger().error(f"[{self.config['name']}] Lost!")
                        previously_connected = False
                    self.joy_msg.axes    = [0.0] * 12
                    self.joy_msg.buttons = [0]   * 24

            if self.is_connected:
                try:
                    while True:
                        event = self.device.read_one()
                        if event is None:
                            break
                        self.process_event(event)
                except (OSError, RuntimeError):
                    self.node.get_logger().error(
                        f"[{self.config['name']}] Read error, reconnecting...")
                    self.is_connected = False
                    self.device = None

            if now - last_joy_time >= 0.1:
                self.joy_msg.header.stamp    = self.ros_clock.now().to_msg()
                self.joy_msg.header.frame_id = "OK" if self.is_connected else "D/C"
                self.joy_pub.publish(self.joy_msg)
                last_joy_time = now

            if now - last_wd_time >= 0.2:
                wd = Bool()
                wd.data = not self.is_connected
                self.wd_pub.publish(wd)
                last_wd_time = now

            time.sleep(0.01)

    def stop(self):
        self.running = False