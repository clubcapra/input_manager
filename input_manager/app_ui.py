import tkinter as tk
from tkinter import filedialog, messagebox
import yaml # Use PyYAML
import os
import time

class InputManagerUI:
    def __init__(self, ros_node, initial_path, on_config_loaded=None):
        self.node = ros_node
        self.on_config_loaded = on_config_loaded
        self.root = tk.Tk()
        self.root.title("Rescue Robot Input Manager")
        self.config_data = {"devices": []}
        
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill='x', padx=10, pady=5)

        tk.Button(btn_frame, text="Import (YAML)", command=self.import_config).pack(side='left', expand=True, fill='x')
        tk.Button(btn_frame, text="Save Config", command=self.save_config).pack(side='left', expand=True, fill='x')
        
        self.device_listbox = tk.Listbox(self.root, font=("Courier", 10))
        self.device_listbox.pack(fill='both', expand=True, padx=10, pady=5)

        tk.Button(self.root, text="Enable / Disable Selected", command=self.toggle_device, bg="#ddd").pack(fill='x', padx=10, pady=10)

        self._load_config(initial_path)

    def _load_config(self, path):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    self.config_data = yaml.safe_load(f) # YAML Loading
                self.update_ui_list()
                self.node.start_devices(self.config_data)
                if self.on_config_loaded: self.on_config_loaded(path)
            except Exception as e:
                messagebox.showerror("YAML Error", f"Failed to parse config: {e}")

    def update_ui_list(self):
        self.device_listbox.delete(0, tk.END)
        for d in self.config_data.get('devices', []):
            self.device_listbox.insert(tk.END, d['name'])

    def toggle_device(self):
        selection = self.device_listbox.curselection()
        if not selection: return
        idx = selection[0]
        device = self.config_data['devices'][idx]
        device['enabled'] = not device.get('enabled', True)
        self.node.start_devices(self.config_data)

    def update_ui_status(self):
        for i, dev in enumerate(self.config_data.get('devices', [])):
            name = dev['name']
            enabled = dev.get('enabled', True)
            worker = self.node.workers.get(name)
            is_alive = worker and worker.is_connected
            
            status = "[OK]" if is_alive else "[LOST]"
            if not enabled: status = "[DISABLED]"
            
            color = "black" if (is_alive and enabled) else "red"
            if not enabled: color = "gray"

            text = f"{status} {name} -> /joy_input/{dev['alias']}"
            if self.device_listbox.get(i) != text:
                self.device_listbox.delete(i)
                self.device_listbox.insert(i, text)
            self.device_listbox.itemconfig(i, fg=color)

    def import_config(self):
        path = filedialog.askopenfilename(filetypes=[("YAML Files", "*.yaml"), ("All", "*.*")])
        if path: self._load_config(path)

    def save_config(self):
        path = filedialog.asksaveasfilename(defaultextension=".yaml")
        if path:
            with open(path, 'w') as f:
                yaml.dump(self.config_data, f, default_flow_style=False)

    def run(self):
        while True:
            try:
                self.update_ui_status()
                self.root.update()
                time.sleep(0.1)
            except: break