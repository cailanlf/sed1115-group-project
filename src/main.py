import math
import json
from time import time_ns, sleep_ms

from boardio import PotentiometerState, ButtonState, ArmController

# Configuration
pot_pin_x = 27
pot_pin_y = 26
pot_poll_interval = 50

# the pin to use for the button
btn_pin = 12

# the time in ms to debounce btn
btn_debounce = 50.

# pins for PWM output to arm
shoulder_pin = 0
elbow_pin = 1
wrist_pin = 2

# offsets for x, y positions
x_offset, y_offset = 0, 0

origin_x, origin_y = 0, 0
shoulder_length, elbow_length = 155, 155

paper_width, paper_height = 215, 300

def solve_kinematics(
    target_x: float, target_y: float,
    origin_x: float, origin_y: float,
    shoulder_length: float, elbow_length: float,
    ) -> 'tuple[float, float] | None':
    """
    Get a solution of (alpha, beta) in degrees to move the arm to the specified position.
    Returns None if there is no solution.
    """
    from math import sqrt, sin, cos, acos, atan2, degrees, atan

    x = target_x - origin_x
    y = target_y - origin_y
    L1, L2 = shoulder_length, elbow_length

    AC = sqrt(
        (x - -50)**2 + (y - 139.7)**2
    )

    # if the target is out of reach, return None
    if AC > L1 + L2 or AC < abs(L1 - L2):
        return None

    angle_CAAbase = atan2(x + 50, 139.7 - y)
    angle_BAC = acos(
        (L1**2 + AC**2 - L2**2) / (2 * L1 * AC)
    )


    alpha = angle_CAAbase - angle_BAC

    beta = acos(
        (L1**2 + L2**2 - AC**2) / (2 * L1 * L2)
    )


    return degrees(alpha), degrees(beta)

def convert_board_coordinates(x: float, y: float) -> 'tuple[float, float]':
    """
    Convert the given [0 - 1] x, y coordinates into coordinates on the board.
    """
    return (x * paper_width, y * paper_height)

def get_actual_angles(arm: 'ArmController') -> 'tuple[float, float]':
    """
    Read the actual angles from the arm and return them.

    Returns:
        A tuple representing the angles that the arm and shoulder are actually at.
    """
    raise NotImplementedError()

CALIBRATION_FILE = "calibration.json"

def load_calibration():
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
            return data["shoulder_offset"], data["elbow_offset"]
    except (OSError, KeyError, ValueError):
        return None

def save_calibration(shoulder_offset, elbow_offset):
    data = {"shoulder_offset": shoulder_offset, "elbow_offset": elbow_offset}
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f)

def main():
    potentiometer_states = PotentiometerState(pot_pin_x, pot_pin_y, pot_poll_interval)
    button_state = ButtonState(btn_pin, btn_debounce)
    arm_controller = ArmController(shoulder_pin=shoulder_pin, elbow_pin=elbow_pin, wrist_pin=wrist_pin)

    # get the offset angles from the board
    # shoulder_offset, elbow_offset = 42, -35
    calibration_data = load_calibration()
    if calibration_data:
        shoulder_offset, elbow_offset = 42, -35
        print(f"Loaded calibration: shoulder offset: {shoulder_offset:.2f}, elbow offset: {elbow_offset:.2f}")
    else:
        print("Calibrating...")
        shoulder_offset, elbow_offset = arm_controller.get_offset_angles()
        save_calibration(shoulder_offset, elbow_offset)
        print(f"New calibration: shoulder offset: {shoulder_offset:.2f}, elbow offset: {elbow_offset:.2f}")

    start = time_ns()
    while True:
        # update time 
        end = time_ns()
        elapsed = (end - start) / 1_000_000
        start = end

        # update the handlers with the elapsed time
        potentiometer_states.update(elapsed)
        button_state.update(elapsed)

        # get the values from the handlers
        x, y = potentiometer_states.get()
        pen_down = button_state.get()

        # convert x, y to board coordinates for the arm
        board_x, board_y = convert_board_coordinates(x, y)

        # solve the inverse kinematics equations
        kinematics_solution = solve_kinematics(
            board_x + x_offset, board_y + y_offset, 
            origin_x, origin_y, 
            shoulder_length, elbow_length
        )

        if kinematics_solution is None:
            # handle this
            sleep_ms(50)
            continue
        
        alpha, beta = kinematics_solution

        # apply alpha, beta offsets
        alpha, beta = alpha + shoulder_offset, beta + elbow_offset
        
        arm_controller.set_arm_angles(alpha, beta)

        # arm_controller.set_wrist_down(pen_down)

        # error_shoulder, error_arm = get_actual_angles(arm_controller)
        print(f"alpha {alpha:.2f}, beta: {beta:.2f}")
        print(f"x: {board_x:.2f}, y: {board_y:.2f}, pen: {'down' if pen_down else 'up'}")
        # sleep_ms(50)

if __name__ == "__main__":
    main()
