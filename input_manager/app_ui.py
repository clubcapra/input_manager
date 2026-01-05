import tkinter as tk
from tkinter import filedialog, messagebox
import json
import os

class InputManagerUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("ROS2 Input Manager")
        self.config_path = "config/default_config.json"
        
        # UI Elements
        self.status_label = tk.Label(self.root, text="Status: Ready", fg="green")
        self.status_label.pack(pady=5)

        self.btn_import = tk.Button(self.root, text="Import Config", command=self.import_config)
        self.btn_import.pack(fill='x', padx=10)

        self.btn_save = tk.Button(self.root, text="Save/Export Config", command=self.export_config)
        self.btn_save.pack(fill='x', padx=10)

        self.device_listbox = tk.Listbox(self.root)
        self.device_listbox.pack(fill='both', expand=True, padx=10, pady=10)

        self.load_initial_config()

    def load_initial_config(self):
        if os.path.exists(self.config_path):
            with open(self.config_path, 'r') as f:
                data = json.load(f)
                self.update_ui_list(data)
                self.node.start_devices(data)

    def import_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if path:
            with open(path, 'r') as f:
                data = json.load(f)
                self.config_path = path
                self.update_ui_list(data)
                self.node.start_devices(data)
                messagebox.showinfo("Success", "Config Loaded")

    def export_config(self):
        # Implementation for saving current UI state back to JSON
        pass

    def update_ui_list(self, data):
        self.device_listbox.delete(0, tk.END)
        for d in data['devices']:
            status = "[ON]" if d['enabled'] else "[OFF]"
            self.device_listbox.insert(tk.END, f"{status} {d['name']} -> {d['topic']}")

    def run(self):
        # Custom loop to handle Tkinter and ROS2
        while True:
            self.root.update_idletasks()
            self.root.update()
            # In a real app, you'd call rclpy.spin_once() here