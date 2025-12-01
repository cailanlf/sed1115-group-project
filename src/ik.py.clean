# =============================================================================
# CLEANUP + ERROR HANDLING:
# - Smoothed potentiometer reads using EMA with clamping.
# - Added robust button debouncing with clear toggle logic.
# - Standardized servo setup and movement methods.
# =============================================================================

from machine import ADC, Pin, PWM
from servo_translator import translate


class PotentiometerState:
    """
    Periodically reads from two potentiometers and caches smoothed values.

    CLEANUP:
        - Uses an Exponential Moving Average (EMA) to reduce noise.
        - Normalizes values to [0.0, 1.0].
    """

    def __init__(self, x_pin: int, y_pin: int, pot_poll_interval_ms: float, alpha: float = 0.15):
        # CLEANUP: store ADC channels as attributes
        self.pot_x = ADC(Pin(x_pin))
        self.pot_y = ADC(Pin(y_pin))

        self.pot_poll_interval = pot_poll_interval_ms
        self.timer = 0.0
        self.alpha = alpha

        # CLEANUP: initialize with first reads so we don't start at 0, 0
        self.x_value = self._read_raw(self.pot_x)
        self.y_value = self._read_raw(self.pot_y)

    def _read_raw(self, adc: ADC) -> float:
        """
        Read a single ADC value and clamp it to [0.0, 1.0].

        ERROR HANDLING:
            - Ensures noisy or out-of-range data cannot propagate.
        """
        val = adc.read_u16() / 65535.0

        # SAFETY: clamp to valid normalized range
        if val < 0.0:
            return 0.0
        if val > 1.0:
            return 1.0
        return val

    def _read(self) -> None:
        """
        Refresh the cached values using EMA smoothing.

        CLEANUP:
            - EMA: new = alpha * raw + (1 - alpha) * old
        """
        raw_x = self._read_raw(self.pot_x)
        raw_y = self._read_raw(self.pot_y)

        self.x_value = self.alpha * raw_x + (1.0 - self.alpha) * self.x_value
        self.y_value = self.alpha * raw_y + (1.0 - self.alpha) * self.y_value

    def get(self) -> "tuple[float, float]":
        """
        Get the cached (x, y) values in [0.0, 1.0].
        """
        return self.x_value, self.y_value

    def update(self, elapsed_time_ms: float) -> None:
        """
        Update the timer and read potentiometers when the interval is reached.

        ERROR HANDLING:
            - Ignores non-positive elapsed times to avoid timer drift.
        """
        if elapsed_time_ms <= 0:
            return

        self.timer += elapsed_time_ms

        if self.timer >= self.pot_poll_interval:
            self.timer = 0.0
            self._read()


class ButtonState:
    """
    Handles reading and debouncing a button, and stores a toggle state.

    CLEANUP:
        - Clear debouncing logic.
        - Uses internal toggle instead of reading raw spikes.
    """

    def __init__(self, btn_pin: int, btn_debounce_ms: float):
        # CLEANUP: configure as pull-up so pressed = 0
        self._btn = Pin(btn_pin, Pin.IN, Pin.PULL_UP)

        self._btn_debounce = btn_debounce_ms
        self._elapsed_time = 0.0

        self._toggled_on = False
        self._btn_last_pressed = False

    def get(self) -> bool:
        """
        Return whether the button is currently toggled on.
        """
        return self._toggled_on

    def update(self, elapsed_time_ms: float) -> None:
        """
        Update the button state with the elapsed time.

        ERROR HANDLING:
            - Protects against non-positive elapsed times.
        """
        if elapsed_time_ms <= 0:
            return

        self._elapsed_time += elapsed_time_ms

        # Button is "pressed" when value() == 0 (because of PULL_UP)
        btn_pressed = (self._btn.value() == 0)

        # CLEANUP: debounced edge detection
        if (
            btn_pressed
            and not self._btn_last_pressed
            and self._elapsed_time >= self._btn_debounce
        ):
            # Registered a clean press → toggle
            self._elapsed_time = 0.0
            self._toggled_on = not self._toggled_on

        self._btn_last_pressed = btn_pressed


class ArmController:
    """
    Simple interface for controlling the shoulder, elbow, and wrist servos.

    CLEANUP:
        - Standardized servo initialization.
        - Centralized angle → PWM conversion.
    """

    def __init__(self, shoulder_pin: int, elbow_pin: int, wrist_pin: int):
        self.shoulder = PWM(Pin(shoulder_pin))
        self.elbow = PWM(Pin(elbow_pin))
        self.wrist = PWM(Pin(wrist_pin))

        # CLEANUP: explicit, consistent frequency setup for all servos
        self.shoulder.freq(50)
        self.elbow.freq(50)
        self.wrist.freq(50)

    def set_arm_angles(self, alpha: float, beta: float) -> None:
        """
        Move the arm to the given shoulder (alpha) and elbow (beta) angles.

        SAFETY:
            - translate() clamps angles so we don't drive servos past their limits.
        """
        shoulder_duty_cycle = translate(alpha)
        elbow_duty_cycle = translate(beta)

        self.shoulder.duty_u16(shoulder_duty_cycle)
        self.elbow.duty_u16(elbow_duty_cycle)

    def set_wrist_down(self, pen_down: bool) -> None:
        """
        Move the wrist to either "pen up" or "pen down" position.
        """
        angle = 30.0 if pen_down else 0.0
        self.wrist.duty_u16(translate(angle))
