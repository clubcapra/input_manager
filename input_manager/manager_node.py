import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from .device_worker import DeviceWorker

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager_node')
        self.workers = {}

    def start_devices(self, config_data):
        """Cleans up old workers and starts new ones based on JSON."""
        self.stop_all_devices()
        
        for dev_cfg in config_data['devices']:
            if dev_cfg.get('enabled', False):
                # Create a specific publisher for this device's topic
                pub = self.create_publisher(Joy, dev_cfg['topic'], 10)
                
                # Create and start the worker thread
                worker = DeviceWorker(dev_cfg, pub)
                worker.start()
                self.workers[dev_cfg['name']] = worker
                self.get_logger().info(f"Started worker for {dev_cfg['name']} on {dev_cfg['topic']}")

    def stop_all_devices(self):
        for name, worker in self.workers.items():
            worker.stop()
            self.get_logger().info(f"Stopped worker: {name}")
        self.workers.clear()