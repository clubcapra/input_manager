import rclpy
import threading
import json
import os
from input_manager.manager_node import InputManagerNode
from input_manager.app_ui import InputManagerUI

SETTINGS_FILE = os.path.expanduser("~/.ros2_input_manager_settings.json")

def get_last_config_path():
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f).get("last_config")
        except: pass
    return "config/default_config.json"

def save_last_config_path(path):
    with open(SETTINGS_FILE, 'w') as f:
        json.dump({"last_config": path}, f)

def main():
    rclpy.init()
    node = InputManagerNode()
    
    # Load persistence
    initial_path = get_last_config_path()

    # Threading for ROS2
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Pass the save callback to the UI so it can update settings on import
    ui = InputManagerUI(node, initial_path, on_config_loaded=save_last_config_path)
    
    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all_devices()
        rclpy.shutdown()

if __name__ == '__main__':
    main()