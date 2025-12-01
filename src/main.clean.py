from math import sqrt
from time import time_ns, sleep_ms

from ik import solve_kinematics
from boardio import PotentiometerState, ButtonState, ArmController

# =============================================================================
# CLEANUP: Centralized configuration values for readability + maintainability
# =============================================================================
POT_PIN_X = 27
POT_PIN_Y = 26
POT_POLL_INTERVAL_MS = 50.0

BTN_PIN = 12
BTN_DEBOUNCE_MS = 50.0

SHOULDER_PIN = 0
ELBOW_PIN = 1
WRIST_PIN = 2

SHOULDER_OFFSET_DEG = 120.0
ELBOW_OFFSET_DEG = 30.0

X_OFFSET = 0.0
Y_OFFSET = 0.0

ORIGIN_X = 0.0
ORIGIN_Y = 0.0
SHOULDER_LENGTH = 155.0
ELBOW_LENGTH = 155.0

PAPER_HEIGHT = 215.0
PAPER_WIDTH = 279.4

VELOCITY_LIMIT = 200.0  # max allowed motion per update

# Use cleaned IK solver
ik_solver = solve_kinematics


# =============================================================================
# CLEANUP: Helper function with input validation + clamping
# =============================================================================
def convert_board_coordinates(x: float, y: float) -> "tuple[float, float]":
    """
    Convert normalized [0.0 - 1.0] coordinates into board coordinates.

    CLEANUP + ERROR HANDLING:
    - Inputs are clamped to prevent out-of-range ADC noise from crashing logic.
    """
    x = max(0.0, min(1.0, x))   # CLEANUP: consistent clamping strategy
    y = max(0.0, min(1.0, y))

    board_x = x * PAPER_WIDTH - PAPER_WIDTH / 2.0
    board_y = y * PAPER_HEIGHT + 50.0
    return board_x, board_y


def get_actual_angles(arm: "ArmController"):
    """
    Placeholder for feedback angles from the hardware.

    CLEANUP: Replaced silent failure with explicit error.
    """
    raise NotImplementedError("Hardware does not support angle feedback yet.")


def main():
    # =============================================================================
    # CLEANUP: Organized object creation clearly + consistently
    # =============================================================================
    potentiometer_states = PotentiometerState(POT_PIN_X, POT_PIN_Y, POT_POLL_INTERVAL_MS)
    button_state = ButtonState(BTN_PIN, BTN_DEBOUNCE_MS)
    arm_controller = ArmController(
        shoulder_pin=SHOULDER_PIN,
        elbow_pin=ELBOW_PIN,
        wrist_pin=WRIST_PIN,
    )

    start = time_ns()
    interp_x, interp_y = 0.0, 0.0

    while True:
        # -------------------------------------------------------------------------
        # CLEANUP: Safe time-step logic with protection against negative intervals
        # -------------------------------------------------------------------------
        end = time_ns()
        elapsed_ms = (end - start) / 1_000_000.0
        start = end

        if elapsed_ms <= 0:
            # ERROR HANDLING: Prevents division-by-zero and unstable motion
            continue

        # Update states
        potentiometer_states.update(elapsed_ms)
        button_state.update(elapsed_ms)

        x, y = potentiometer_states.get()
        pen_down = button_state.get()

        # Convert joystick input → board coordinates
        board_x, board_y = convert_board_coordinates(x, y)

        # -------------------------------------------------------------------------
        # CLEANUP: Safer velocity limiting with no divide-by-zero risk
        # -------------------------------------------------------------------------
        diff_x = board_x - interp_x
        diff_y = board_y - interp_y
        dist = sqrt(diff_x ** 2 + diff_y ** 2)

        # ---------------------------------------------------------------------
        # ERROR HANDLING ADDED BY DANELLA:
        # Prevent division-by-zero when the interpolated point is already
        # at the target (dist == 0). In that case, we simply don't move.
        # ---------------------------------------------------------------------
        if dist == 0:
            dir_x, dir_y = 0.0, 0.0
        else:
            dir_x = diff_x / dist
            dir_y = diff_y / dist

        step = min(VELOCITY_LIMIT, dist)
        interp_x += dir_x * step
        interp_y += dir_y * step

        # -------------------------------------------------------------------------
        # ERROR HANDLING: Protect IK solver from invalid or failing computations
        # -------------------------------------------------------------------------
        try:
            kinematics_solution = ik_solver(
                interp_x + X_OFFSET,
                interp_y + Y_OFFSET,
                ORIGIN_X,
                ORIGIN_Y,
                SHOULDER_LENGTH,
                ELBOW_LENGTH,
            )
        except Exception as e:
            print(f"[ERROR] IK failed for ({interp_x:.2f}, {interp_y:.2f}): {e}")
            sleep_ms(50)
            continue

        if kinematics_solution is None:
            # CLEANUP: Clear warning instead of silent failure
            print(f"[WARN] Target ({interp_x:.2f}, {interp_y:.2f}) unreachable.")
            sleep_ms(50)
            continue

        # Apply offsets
        alpha, beta = kinematics_solution
        alpha = alpha + SHOULDER_OFFSET_DEG
        beta = 180.0 - (beta + ELBOW_OFFSET_DEG)

        # -------------------------------------------------------------------------
        # CLEANUP: Explicit, readable movement commands
        # -------------------------------------------------------------------------
        arm_controller.set_arm_angles(alpha, beta)
        arm_controller.set_wrist_down(pen_down)

        # Debug output
        print(
            f"x={board_x:.2f}, y={board_y:.2f}, pen={'down' if pen_down else 'up'} | "
            f"alpha={alpha:.2f}, beta={beta:.2f}"
        )

        sleep_ms(50)


if __name__ == "__main__":
    main()
