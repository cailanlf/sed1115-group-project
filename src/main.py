from time import time_ns

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

def solve_kinematics(x: float, y: float) -> 'tuple[float, float] | None':
    """
    Get a solution of (alpha, beta) in degrees to move the arm to the specified position.
    Returns None if there is no solution.
    """
    raise NotImplementedError()

def convert_board_coordinates(x: float, y: float) -> 'tuple[float, float]':
    """
    Convert the given [0 - 1] x, y coordinates into coordinates on the board.
    """
    raise NotImplementedError()

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

        print(x, y, f"x: {x:.f4}, y: {y:.f4}, pen: {'down' if pen_down else 'up'}")

        # convert x, y to board coordinates for the arm
        board_x, board_y = convert_board_coordinates(x, y)

        # solve the inverse kinematics equations
        kinematics_solution = solve_kinematics(x, y)

        if kinematics_solution is None:
            # handle this
            raise NotImplementedError()
            continue
        
        alpha, beta = kinematics_solution
        
        arm_controller.set_arm_angles(alpha, beta)
        arm_controller.set_wrist_down(pen_down)

        error_shoulder, error_arm = get_actual_angles(arm_controller)

if __name__ == "__main__":
    main()
