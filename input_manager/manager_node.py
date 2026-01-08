from rclpy.node import Node
from .device_worker import DeviceWorker

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager_node')
        self.workers = {}

    def start_devices(self, config_data):
        self.stop_all_devices()
        for dev_cfg in config_data.get('devices', []):
            if dev_cfg.get('enabled', False):
                worker = DeviceWorker(dev_cfg, self)
                worker.start()
                self.workers[dev_cfg['name']] = worker
                self.get_logger().info(f"Launched worker: {dev_cfg['name']} (Alias: {dev_cfg['alias']})")

    def stop_all_devices(self):
        for name, worker in self.workers.items():
            worker.stop()
        self.workers.clear()