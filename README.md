# input_manager

ROS2 Humble package for reading gamepad input via `evdev` and publishing standardized `sensor_msgs/Joy` messages. Built for CAPRA's rescue robot.

---

## Design

The package runs one thread per device. Each **DeviceWorker** thread owns a single controller — it finds the device, reads raw `evdev` events in a non-blocking loop, normalizes them, and publishes at a fixed rate regardless of whether input is happening. If a device disconnects, the worker keeps retrying in the background and reconnects automatically.

A **manager node** owns all workers and handles starting/stopping them when a config is loaded. A lightweight **Tkinter UI** sits on top for config loading and live status. All of this is driven by a YAML config that fully describes each device: its evdev ID, axis normalization ranges, and any axis-to-button conversions.

**Topics published per device:**
- `/joy_input/<alias>` — `sensor_msgs/Joy` at 10 Hz
- `/input_watchdog/<alias>` — `std_msgs/Bool` at 5 Hz (`true` = disconnected)

---

## Package Structure

```
input_manager_package/
├── input_manager/          # Python package — all node logic
│   ├── main.py             # Entry point, CLI args, GUI/headless mode
│   ├── manager_node.py     # ROS2 node, owns all DeviceWorkers
│   ├── device_worker.py    # Per-device thread: evdev → Joy publisher
│   ├── app_ui.py           # Tkinter UI
│   └── evdev_code_tester.py  # Standalone utility, not part of the node
├── launch/
│   └── input_manager.launch.py
├── config/
│   └── default_config.yaml
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Building

```bash
cd ~/your_ws
colcon build --packages-select input_manager
source install/setup.bash
```

---

## Usage

### GUI mode (default)

```bash
ros2 launch input_manager input_manager.launch.py
```

The UI shows all devices declared in the active config with their live connection status. From the UI you can:
- **Import (YAML)** — load a different config file, restarts all workers immediately
- **Save Config** — export the current config to a file
- **Enable / Disable Selected** — toggle a device without editing the config

The last loaded config path is remembered across sessions (`~/.ros2_input_manager_settings.json`).

### Headless mode

```bash
ros2 launch input_manager input_manager.launch.py no_gui:=true
```

Prints a live one-line status to the console. Ctrl+C to exit.

### Loading a specific config

```bash
# Via launch argument
ros2 launch input_manager input_manager.launch.py config:=/path/to/your_config.yaml

# Or directly
ros2 run input_manager input_manager --ros-args -p config:=/path/to/your_config.yaml -p no_gui:=true
```

The default config is the one installed with the package at `share/input_manager/config/default_config.yaml`. During development it's easier to point directly at your working copy with `--config`.

---

## Config Reference

```yaml
devices:
  - name: "My Controller"
    udev_path: "/dev/input/my_symlink"  # preferred — set up a udev rule
    id: "045e:028e"                     # fallback vendor:product scan
    alias: "my_controller"              # used in topic names
    enabled: true
    sanitize: false                     # enables deadzone
    deadzone: 0.0                       # in [-1, 1] space, applied after normalization
    mapping:
      buttons:
        <joy_index>: <evdev_code>
      axes:
        <joy_index>: <evdev_code>
      axis_ranges:                      # optional — overrides default /32767 normalization
        <evdev_code>: [raw_min, raw_max]  # raw integers evdev actually sends
      axes_as_buttons:                  # optional — mirror an axis to button slots
        - axis_code: <evdev_code>
          neg_button: <joy_index>       # fires when val < -threshold (-1 to skip)
          pos_button: <joy_index>       # fires when val > +threshold (-1 to skip)
          threshold: 0.5
```

**`axis_ranges` explained:** by default all axes are normalized by dividing the raw evdev integer by 32767 (correct for analog sticks). For axes that report a different range, declare the actual raw extremes:
- D-PAD axes send `-1 / 0 / 1` → declare `[-1, 1]`
- XBOX triggers send `0 to 1023` → declare `[0, 1023]`

Use `evdev_code_tester` to find the raw values your hardware actually sends:
```bash
ros2 run input_manager evdev_tester
```
