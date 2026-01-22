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
    
    # Map file descriptors to device objects and store their ID strings
    dev_map = {}
    for d in devices:
        # Create the xxxx:xxxx ID string once
        hw_id = f"{d.info.vendor:04x}:{d.info.product:04x}"
        dev_map[d.fd] = {
            "device": d,
            "hw_id": hw_id
        }
        print(f"Device: {d.name:20} | ID: {hw_id} | Path: {d.path}")
    
    print("\nPress buttons or move sticks. (Ctrl+C to exit)")
    print("-" * 80)

    # Dictionary to track unique codes per device ID + event type + code
    seen_inputs = set()

    try:
        while True:
            # Efficiently wait for any device to have data
            r, w, x = select.select(dev_map.keys(), [], [])
            
            for fd in r:
                current_dev = dev_map[fd]["device"]
                hw_id = dev_map[fd]["hw_id"]
                
                for event in current_dev.read():
                    # type 1 = BTN (EV_KEY), type 3 = ABS (EV_ABS/Axes)
                    if event.type in [1, 3]:
                        # Create a unique key including the hardware ID
                        unique_id = f"{hw_id}_{event.type}_{event.code}"
                        
                        if unique_id not in seen_inputs:
                            seen_inputs.add(unique_id)
                            
                            label = "BUTTON" if event.type == 1 else "AXIS"
                            print(f"[NEW] ID: {hw_id} | {label:6} | Code: {event.code:4} | Device: {current_dev.name}")

    except KeyboardInterrupt:
        print("\nExiting mapper.")

if __name__ == "__main__":
    main()