import rclpy
import threading
import json
import os
from input_manager.manager_node import InputManagerNode
from input_manager.app_ui import InputManagerUI

# This file remains JSON as it's a simple key-value store for app state
SETTINGS_FILE = os.path.expanduser("~/.ros2_input_manager_settings.json")

def get_last_config_path():
    """Retrieves the last used YAML config path."""
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                path = json.load(f).get("last_config")
                # Verify the file still exists on disk
                if path and os.path.exists(path):
                    return path
        except Exception:
            pass
    # Default fallback to a yaml file
    return "config/default_config.yaml"

def save_last_config_path(path):
    """Saves the current YAML config path for the next session."""
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump({"last_config": path}, f)
    except Exception as e:
        print(f"Failed to save settings: {e}")

def main():
    rclpy.init()
    
    # Initialize the ROS2 Node
    node = InputManagerNode()
    
    # 1. Determine which config to load
    initial_path = get_last_config_path()

    # 2. Start ROS2 spinning in a background thread
    # This allows the UI to stay responsive while ROS handles communication
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 3. Launch the UI
    # We pass the node, the initial path, and the persistence callback
    ui = InputManagerUI(node, initial_path, on_config_loaded=save_last_config_path)
    
    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: Stop all worker threads and shutdown ROS2
        node.stop_all_devices()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()