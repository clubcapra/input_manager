import rclpy
from rclpy.node import Node
from .device_worker import DeviceWorker

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager_node')
        self.workers = {}

    def start_devices(self, config_data):
        self.stop_all_devices()
        for dev_cfg in config_data['devices']:
            if dev_cfg['enabled']:
                pub = self.create_publisher(Joy, dev_cfg['topic'], 10)
                worker = DeviceWorker(dev_cfg, pub)
                worker.start()
                self.workers[dev_cfg['name']] = worker

    def stop_all_devices(self):
        for worker in self.workers.values():
            worker.stop()
        self.workers.clear()