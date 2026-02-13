import rclpy
import threading
import json
import os
import argparse
import time
from manager_node import InputManagerNode
from app_ui import InputManagerUI

SETTINGS_FILE = os.path.expanduser("~/.ros2_input_manager_settings.json")

def get_last_config_path():
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                path = json.load(f).get("last_config")
                if path and os.path.exists(path):
                    return path
        except Exception:
            pass
    return "config/default_config.yaml"

def save_last_config_path(path):
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump({"last_config": path}, f)
    except Exception as e:
        print(f"Failed to save settings: {e}")

def run_headless(node, config_path):
    """Simple console monitor for NO-GUI mode."""
    import yaml
    if not os.path.exists(config_path):
        node.get_logger().error(f"Config file not found: {config_path}")
        return

    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    node.start_devices(config_data)
    node.get_logger().info("Running in HEADLESS mode. Press Ctrl+C to exit.")
    
    try:
        while rclpy.ok():
            # Create a status summary line
            status_reports = []
            for name, worker in node.workers.items():
                state = "OK" if worker.is_connected else "LOST"
                status_reports.append(f"{name}: [{state}]")
            
            # Print status over the same line to avoid scrolling spam
            if status_reports:
                print(f"\rStatus: {' | '.join(status_reports)}", end="", flush=True)
            
            time.sleep(1.0) # Update console every second
    except KeyboardInterrupt:
        pass

def main():
    parser = argparse.ArgumentParser(description="Rescue Robot Input Manager")
    parser.add_argument('--no-gui', '-n', action='store_true', help="Run without the Tkinter interface")
    parser.add_argument('--config', '-c', type=str, help="Path to a specific YAML config")
    args = parser.parse_args()

    rclpy.init()
    node = InputManagerNode()
    
    # Priority: Command line arg -> Persistence -> Default
    initial_path = args.config if args.config else get_last_config_path()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    if args.no_gui:
        run_headless(node, initial_path)
    else:
        node.get_logger().info("Input Manager Service Started (GUI Mode).")
        ui = InputManagerUI(node, initial_path, on_config_loaded=save_last_config_path)
        try:
            ui.run()
        except KeyboardInterrupt:
            pass

    # Cleanup
    node.get_logger().info("Shutting down...")
    node.stop_all_devices()
    rclpy.shutdown()

if __name__ == '__main__':
    main()