import threading
from evdev import InputDevice, list_devices, ecodes
from sensor_msgs.msg import Joy

class DeviceWorker(threading.Thread):
    def __init__(self, config, publisher):
        super().__init__(daemon=True)
        self.config = config
        self.publisher = publisher
        self.running = True
        self.device = self._find_device()

    def _find_device(self):
        devices = [InputDevice(path) for path in list_devices()]
        for dev in devices:
            vid_pid = f"{dev.info.vendor:04x}:{dev.info.product:04x}"
            if vid_pid == self.config['id']:
                # Optional: Add serial check here if hardware supports it
                return dev
        return None

    def run(self):
        if not self.device:
            print(f"Device {self.config['name']} not found.")
            return

        # Simple mapping logic
        axes = [0.0] * 8
        buttons = [0] * 11

        for event in self.device.read_loop():
            if not self.running: break
            
            if event.type == ecodes.EV_ABS:
                # Map absolute axes to 0.0 - 1.0 range
                # This is a simplified example; actual mapping depends on controller
                pass 
            
            # Publish Joy Message
            msg = Joy()
            msg.header.stamp = threading.Event()._timestamp() # Simplified
            msg.axes = axes
            msg.buttons = buttons
            self.publisher.publish(msg)

    def stop(self):
        self.running = False