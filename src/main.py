from math import sqrt
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

# offsets for angles and x, y positions
shoulder_offset, elbow_offset = 120, 30
x_offset, y_offset = 0, 0

origin_x, origin_y = 0, 0
shoulder_length, elbow_length = 155, 155

paper_height, paper_width = 215, 279.4

def solve_kinematics(
    target_x: float, target_y: float,
    origin_x: float, origin_y: float,
    shoulder_length: float, elbow_length: float,
    ) -> 'tuple[float, float] | None':
    """
    Get a solution of (alpha, beta) in degrees to move the arm to the specified position.
    Returns None if there is no solution.
    """
    from math import sqrt, sin, cos, acos, atan2, degrees

    x = target_x - origin_x
    y = target_y - origin_y
    L1, L2 = shoulder_length, elbow_length

    AC = sqrt(x*x + y*y)

    # if the target is out of reach, return None
    if AC > L1 + L2 or AC < abs(L1 - L2):
        return None

    angle_CAX = atan(y/x)
    angle_BAC = acos(
        (L1**2 + AC**2 - L2**2) / (2 * L1 * AC)
    )

    alpha = angle_CAX - angle_BAC
    beta = acos(
        (L1**2 + L2**2 - AC**2) / (2 * L1 * L2)
    )


    return degrees(alpha), degrees(beta)

def convert_board_coordinates(x: float, y: float) -> 'tuple[float, float]':
    """
    Convert the given [0 - 1] x, y coordinates into coordinates on the board.
    """
    return (x * paper_width - paper_width/2, y * paper_height + 50)

def get_actual_angles(arm: 'ArmController') -> 'tuple[float, float]':
    """
    Read the actual angles from the arm and return them.

    Returns:
        A tuple representing the angles that the arm and shoulder are actually at.
    """
    raise NotImplementedError()

def main():
    potentiometer_states = PotentiometerState(pot_pin_x, pot_pin_y, pot_poll_interval)
    button_state = ButtonState(btn_pin, btn_debounce)
    arm_controller = ArmController(shoulder_pin=shoulder_pin, elbow_pin=elbow_pin, wrist_pin=wrist_pin)

    start = time_ns()

    # limit the X, Y movement to a certain velocity
    interp_x, interp_y = 0, 0

    # in u/s
    velocity_limit = 200.0
    
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
        # these are our target angles
        board_x, board_y = convert_board_coordinates(x, y)

        # get a vector of the differences
        diff_x, diff_y = board_x - interp_x, board_y - interp_y

        dist = sqrt(diff_x ** 2 + diff_y ** 2)

        # get the magnitude
        velocity = dist * elapsed

        # make unit vector
        diff_x, diff_y = diff_x / velocity, diff_y / velocity

        # move interp position up to max velocity
        interp_x, interp_y = interp_x + diff_x * min(velocity_limit, velocity), interp_y + diff_y * min(velocity_limit, velocity) 

        # solve the inverse kinematics equations
        kinematics_solution = solve_kinematics(
            interp_x + x_offset, interp_y + y_offset, 
            origin_x, origin_y, 
            shoulder_length, elbow_length
        )

        if kinematics_solution is None:
            # handle this
            sleep_ms(50)
            continue
        
        alpha, beta = kinematics_solution

        # apply alpha, beta offsets
        alpha, beta = alpha + shoulder_offset, 180 - (beta + elbow_offset)

        print(alpha, beta)
        
        arm_controller.set_arm_angles(alpha, beta)

        # arm_controller.set_wrist_down(pen_down)

        # error_shoulder, error_arm = get_actual_angles(arm_controller)
        print(board_x, board_y, f"x: {board_x:.4f}, y: {board_y:.4f}, pen: {'down' if pen_down else 'up'}")
        sleep_ms(50)

if __name__ == "__main__":
    main()
