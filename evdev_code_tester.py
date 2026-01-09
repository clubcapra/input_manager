import evdev
from evdev import InputDevice, list_devices
import select
import sys

def main():
    # 1. Find all available input devices
    devices = [InputDevice(path) for path in list_devices()]
    
    if not devices:
        print("No input devices found. Check permissions (sudo?)")
        return

    print(f"--- Listening to {len(devices)} devices ---")
    for d in devices:
        print(f"Device: {d.name} | Path: {d.path} | ID: {d.info.vendor:04x}:{d.info.product:04x}")
    
    print("\nPress buttons or move sticks. (Ctrl+C to exit)")
    print("-" * 50)

    # Dictionary to track unique codes per device to prevent spam
    seen_inputs = set()
    
    # Map file descriptors to device objects
    dev_map = {d.fd: d for d in devices}

    try:
        while True:
            # Efficiently wait for any device to have data
            r, w, x = select.select(dev_map.keys(), [], [])
            
            for fd in r:
                for event in dev_map[fd].read():
                    # type 1 = BTN, type 3 = ABS (Axes)
                    if event.type in [1, 3]:
                        dev_name = dev_map[fd].name
                        # Create a unique key for this specific input on this device
                        unique_id = f"{dev_name}_{event.type}_{event.code}"
                        
                        if unique_id not in seen_inputs:
                            seen_inputs.add(unique_id)
                            
                            label = "BUTTON" if event.type == 1 else "AXIS"
                            print(f"[NEW] {dev_name:20} | {label:6} | Code: {event.code:3} | Raw Val: {event.value}")

    except KeyboardInterrupt:
        print("\nExiting mapper.")

if __name__ == "__main__":
    main()