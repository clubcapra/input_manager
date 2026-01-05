import rclpy
import threading
from input_manager.manager_node import InputManagerNode
from input_manager.app_ui import InputManagerUI

def main():
    rclpy.init()
    node = InputManagerNode()
    
    # Run ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run UI in the main thread
    ui = InputManagerUI(node)
    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all_devices()
        rclpy.shutdown()

if __name__ == '__main__':
    main()