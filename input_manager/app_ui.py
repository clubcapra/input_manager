import tkinter as tk
from tkinter import filedialog, messagebox
import json
import os

class InputManagerUI:
    def __init__(self, ros_node, initial_path, on_config_loaded=None):
        self.node = ros_node
        self.on_config_loaded = on_config_loaded
        self.root = tk.Tk()
        self.root.title("Rescue Robot Input Manager")
        self.config_data = {"devices": []}
        
        # --- UI Layout ---
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill='x', padx=10, pady=5)

        tk.Button(btn_frame, text="Import", command=self.import_config).pack(side='left', expand=True, fill='x')
        tk.Button(btn_frame, text="Save Config", command=self.save_config).pack(side='left', expand=True, fill='x')
        
        self.device_listbox = tk.Listbox(self.root, font=("Courier", 10), selectmode=tk.SINGLE)
        self.device_listbox.pack(fill='both', expand=True, padx=10, pady=5)

        # The Toggle Button
        self.btn_toggle = tk.Button(self.root, text="Enable / Disable Selected", command=self.toggle_device, bg="#ddd")
        self.btn_toggle.pack(fill='x', padx=10, pady=10)

        self._load_config(initial_path)

    def toggle_device(self):
        """Flips the 'enabled' state for the selected device and restarts workers."""
        selection = self.device_listbox.curselection()
        if not selection:
            return
        
        idx = selection[0]
        device = self.config_data['devices'][idx]
        
        # Flip state
        device['enabled'] = not device.get('enabled', True)
        
        # Immediately push new state to the ROS node
        self.node.start_devices(self.config_data)
        self.update_ui_status()

    def update_ui_status(self):
        """Updates colors and text based on connection AND enabled state."""
        for i, dev in enumerate(self.config_data.get('devices', [])):
            name = dev['name']
            enabled = dev.get('enabled', True)
            worker = self.node.workers.get(name)
            
            is_alive = worker and worker.is_connected
            
            # Formatting
            prefix = "[OK]" if is_alive else "[LOST]"
            if not enabled:
                prefix = "[DISABLED]"
            
            color = "black" if (is_alive and enabled) else "red"
            if not enabled: color = "gray"

            display_text = f"{prefix} {name} -> {dev['topic']}"
            
            if self.device_listbox.get(i) != display_text:
                self.device_listbox.delete(i)
                self.device_listbox.insert(i, display_text)
            
            self.device_listbox.itemconfig(i, fg=color)

    def save_config(self):
        """Save As: Never overwrites silently."""
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json")],
            initialfile="new_config.json"
        )
        if path:
            try:
                with open(path, 'w') as f:
                    json.dump(self.config_data, f, indent=4)
                messagebox.showinfo("Success", "Configuration saved.")
            except Exception as e:
                messagebox.showerror("Error", f"Save failed: {e}")

    def _load_config(self, path):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    self.config_data = json.load(f)
                self.device_listbox.delete(0, tk.END)
                for d in self.config_data['devices']:
                    self.device_listbox.insert(tk.END, d['name'])
                self.node.start_devices(self.config_data)
                if self.on_config_loaded:
                    self.on_config_loaded(path)
            except Exception as e:
                print(f"Load error: {e}")

    def import_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if path: self._load_config(path)

    def run(self):
        while True:
            try:
                self.update_ui_status()
                self.root.update_idletasks()
                self.root.update()
                import time
                time.sleep(0.1)
            except: break