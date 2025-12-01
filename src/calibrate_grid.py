import json
import time
from boardio import PotentiometerState, ButtonState, ArmController

# Configuration
pot_pin_x = 27
pot_pin_y = 26
pot_poll_interval = 0.05 # seconds

btn_pin = 22
btn_debounce = 0.2 # seconds

shoulder_pin = 0
elbow_pin = 1
wrist_pin = 2

CALIBRATION_FILE = "grid_calibration.json"

def save_grid_calibration(data):
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f)
    print(f"Saved {len(data)} points to {CALIBRATION_FILE}")

def main():
    print("Starting Grid Calibration...")
    print("Controls:")
    print("  - Use potentiometers to move the arm.")
    print("  - Press the button to record a point.")
    print("  - You will be prompted to enter X Y coordinates (cm).")
    print("  - Press Ctrl+C to exit and save.")

    # Initialize hardware
    # Note: boardio classes use ms for time, but we might want to check if we can use seconds or need to convert.
    # Looking at boardio.py, PotentiometerState takes pot_poll_interval (float), update takes elapsed_time (float).
    # ButtonState takes btn_debounce (float).
    # It seems they expect consistent units. main.py uses time_ns() and converts to ms (elapsed / 1_000_000).
    # Let's stick to ms to be safe and consistent with main.py.
    
    pot_poll_interval_ms = 50
    btn_debounce_ms = 200

    pot_state = PotentiometerState(pot_pin_x, pot_pin_y, pot_poll_interval_ms)
    btn_state = ButtonState(btn_pin, btn_debounce_ms)
    arm = ArmController(shoulder_pin, elbow_pin, wrist_pin)

    calibration_data = {} # Format: "x,y": [angle1, angle2]

    start_time = time.ticks_ms()

    try:
        while True:
            current_time = time.ticks_ms()
            elapsed = time.ticks_diff(current_time, start_time)
            start_time = current_time

            pot_state.update(elapsed)
            btn_state.update(elapsed)

            # 1. Read Potentiometers & Map to Angles
            # PotentiometerState.get() returns 0.0 to 1.0
            val_x, val_y = pot_state.get()
            
            # Map 0-1 to 0-180 degrees
            angle_shoulder = val_x * 180
            angle_elbow = val_y * 180

            # 2. Move Arm (Forward Kinematics)
            arm.set_arm_angles(angle_shoulder, angle_elbow)

            # 3. Check Button for Recording
            if btn_state.get():
                # Debounce logic is inside ButtonState, but get() returns toggle state.
                # We want a trigger. ButtonState implementation in boardio.py:
                # get() returns self._toggled_on. This toggles on/off.
                # This might not be ideal for a "trigger" event.
                # Let's look at boardio.py again.
                # It seems ButtonState is designed for toggle switches (pen up/down).
                # For a momentary trigger, we might need to check if the state CHANGED or implement a simple check here.
                # However, since we can't easily modify boardio.py without affecting main.py, 
                # let's just wait for the user to toggle it "ON" to record, then they have to toggle it "OFF" to continue?
                # Or better: just read the pin directly for this script since it's a standalone calibration tool.
                pass

            # Alternative: Just use a simple blocking input check if we want to be safe, 
            # but we need the loop to keep running to control the servos.
            # Let's use the ButtonState but detect a change.
            
            # Actually, looking at boardio.py, ButtonState.get() returns _toggled_on.
            # If we want a "click", we can watch for it to flip.
            
            # For simplicity in this script, let's just poll the button pin directly with a simple debounce 
            # because we want to pause the loop when pressed.
            from machine import Pin
            btn = Pin(btn_pin, Pin.IN, Pin.PULL_DOWN) 
            if btn.value():
                # Button pressed
                print("\nButton pressed! Pausing...")
                
                # Wait for release
                while btn.value():
                    time.sleep(0.1)
                
                # Prompt user
                try:
                    coord_str = input("Enter coordinates 'X Y' (or 's' to skip, 'q' to quit): ").strip()
                    if coord_str.lower() == 'q':
                        break
                    if coord_str.lower() == 's':
                        print("Skipped.")
                        continue
                    
                    parts = coord_str.split()
                    if len(parts) == 2:
                        x_cm = float(parts[0])
                        y_cm = float(parts[1])
                        
                        key = f"{x_cm},{y_cm}"
                        calibration_data[key] = [angle_shoulder, angle_elbow]
                        print(f"Recorded: {key} -> [{angle_shoulder:.2f}, {angle_elbow:.2f}]")
                    else:
                        print("Invalid format. Expected 'X Y'.")
                except ValueError:
                    print("Invalid numbers.")
                
                print("Resuming...")
                start_time = time.ticks_ms() # Reset timer to avoid huge elapsed jump

            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("\nExiting...")

    save_grid_calibration(calibration_data)

if __name__ == "__main__":
    main()
