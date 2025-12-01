from machine import ADC, Pin, PWM
from servo_translator import translate

# =============================================================================
# CLEANUP + ERROR HANDLING MODULE
# - Smoothed potentiometer readings using EMA.
# - Added clamping and validation to prevent invalid values.
# - Improved button debouncing and clarified toggle behavior.
# - Standardized servo setup and implemented safe wrist movement.
# =============================================================================


class PotentiometerState:
    """
    Handles periodically reading from the potentiometers and caching the values.

    CLEANUP:
        - Normalizes readings to [0.0, 1.0].
        - Uses exponential moving average (EMA) to smooth noise.
    """

    pot_x: ADC
    pot_y: ADC

    x_value: float
    y_value: float

    pot_poll_interval: float  # in ms
    timer: float
    alpha: float  # smoothing factor

    def __init__(self, x_pin: int, y_pin: int, pot_poll_interval: float, alpha: float = 0.15):
        # CLEANUP: explicitly configure ADC channels
        self.pot_x = ADC(Pin(x_pin))
        self.pot_y = ADC(Pin(y_pin))

        self.pot_poll_interval = pot_poll_interval
        self.timer = 0.0
        self.alpha = alpha

        # CLEANUP: initialize from an initial read instead of (0, 0)
        self.x_value = self._read_raw(self.pot_x)
        self.y_value = self._read_raw(self.pot_y)

    # -------------------------------------------------------------------------
    # INTERNAL HELPERS
    # -------------------------------------------------------------------------
    def _read_raw(self, adc: ADC) -> float:
        """
        Read a single ADC value and clamp it to [0.0, 1.0].

        ERROR HANDLING:
            - Prevents any out-of-range values from propagating.
        """
        val = adc.read_u16() / 65535.0

        # Clamp to [0.0, 1.0]
        if val < 0.0:
            return 0.0
        if val > 1.0:
            return 1.0
        return val

    def _read(self) -> None:
        """
        Read the values from the potentiometers and update internal state.

        CLEANUP:
            - Uses EMA for smoothing:
              new = alpha * raw + (1 - alpha) * old
        """
        raw_x = self._read_raw(self.pot_x)
        raw_y = self._read_raw(self.pot_y)

        self.x_value = self.alpha * raw_x + (1.0 - self.alpha) * self.x_value
        self.y_value = self.alpha * raw_y + (1.0 - self.alpha) * self.y_value

    # -------------------------------------------------------------------------
    # PUBLIC API
    # -------------------------------------------------------------------------
    def get(self) -> "tuple[float, float]":
        """
        Get the cached (x, y) values as a tuple in [0.0, 1.0].
        """
        return self.x_value, self.y_value

    def update(self, elapsed_time: float) -> None:
        """
        Update internal timer and read when the polling interval is reached.

        ERROR HANDLING:
            - Ignores non-positive elapsed times to avoid weird timing behavior.
        """
        if elapsed_time <= 0:
            return

        self.timer += elapsed_time

        if self.timer >= self.pot_poll_interval:
            self.timer = 0.0
            self._read()


class ButtonState:
    """
    Handles reading from and debouncing a button and caches the button's toggle state.

    CLEANUP:
        - Uses pull-up configuration so pressed = 0.
        - Implements clear debounced toggle behavior.
    """

    _btn: Pin
    _btn_debounce: float

    _elapsed_time: float

    _toggled_on: bool
    _btn_last_pressed: bool

    def __init__(self, btn_pin: int, btn_debounce: float):
        # CLEANUP: configure the button with PULL_UP so logic is deterministic
        self._btn = Pin(btn_pin, Pin.IN, Pin.PULL_UP)
        self._btn_debounce = btn_debounce

        self._elapsed_time = 0.0
        self._toggled_on = False
        self._btn_last_pressed = False

    def get(self) -> bool:
        """
        Retrieve whether the button is currently toggled or not.
        """
        return self._toggled_on

    def update(self, elapsed_time: float) -> None:
        """
        Update the state with the elapsed time.

        ERROR HANDLING:
            - Skips update when elapsed_time is not valid.
        """
        if elapsed_time <= 0:
            return

        self._elapsed_time += elapsed_time

        # get the button's current pressed state (pressed = 0 because of PULL_UP)
        btn_pressed = (self._btn.value() == 0)

        # CLEANUP: proper debouncing and rising-edge detection
        if (
            btn_pressed
            and not self._btn_last_pressed
            and self._elapsed_time >= self._btn_debounce
        ):
            self._elapsed_time = 0.0
            self._toggled_on = not self._toggled_on

        self._btn_last_pressed = btn_pressed


class ArmController:
    """
    Controls the shoulder, elbow, and wrist servos.

    CLEANUP:
        - Centralizes servo frequency configuration.
        - Uses translate() for angle → PWM conversion.
    """

    shoulder: PWM
    elbow: PWM
    wrist: PWM

    def __init__(self, shoulder_pin: int, elbow_pin: int, wrist_pin: int):
        # CLEANUP: consistent 50 Hz for all servos
        self.shoulder = PWM(Pin(shoulder_pin))
        self.elbow = PWM(Pin(elbow_pin))
        self.wrist = PWM(Pin(wrist_pin))

        self.shoulder.freq(50)
        self.elbow.freq(50)
        self.wrist.freq(50)

    def set_arm_angles(self, alpha: float, beta: float) -> None:
        """
        Move the arm to a specific position.

        CLEANUP:
            - Uses translate() which already clamps angles safely to [0, 180].
        """

        # original mapping preserved:
        shoulder_angle = 90.0 - alpha
        elbow_angle = beta

        shoulder_duty_cycle = translate(shoulder_angle)
        elbow_duty_cycle = translate(elbow_angle)

        self.shoulder.duty_u16(shoulder_duty_cycle)
        self.elbow.duty_u16(elbow_duty_cycle)

    def move_wrist(self, pen_down: bool) -> None:
        """
        Set the pen to down or up.

        CLEANUP:
            - Implemented instead of raising NotImplementedError.
        SAFETY:
            - Uses translate() so angles are clamped and safe.
        """
        # small angle for "up", larger angle for "down" (tunable)
        angle = 30.0 if pen_down else 0.0
        duty = translate(angle)
        self.wrist.duty_u16(duty)
