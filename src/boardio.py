from machine import ADC, Pin, PWM
from servo_translator import translate

class PotentiometerState:
    """
    Handles periodically reading from the potentiometers and caching the values.
    """
    pot_x: ADC
    pot_y: ADC

    x_value: float
    y_value: float
    
    pot_poll_interval: float
    timer: float
    alpha: float

    def __init__(self, x_pin: int, y_pin: int, pot_poll_interval: float, alpha: float = 0.15):
        self.pot_x = ADC(Pin(x_pin))
        self.pot_y = ADC(Pin(y_pin))

        self.pot_poll_interval = pot_poll_interval
        self.timer = 0
        self.alpha = alpha
        
        # Initialize with a first read so we don't start at 0
        self.x_value = self._read_raw(self.pot_x)
        self.y_value = self._read_raw(self.pot_y)

    def _read_raw(self, adc: ADC) -> float:
        """
        Helper to read and clamp a single ADC value.
        """
        val = adc.read_u16() / 65535.0
        return 0. if val < 0. else 1. if val > 1. else val

    def _read(self):
        """
        Read the values from the potentiometers and update the internal state using EMA smoothing.
        """
        # Read raw values
        raw_x = self._read_raw(self.pot_x)
        raw_y = self._read_raw(self.pot_y)

        # Apply Exponential Moving Average (EMA)
        # New = alpha * raw + (1 - alpha) * old
        self.x_value = self.alpha * raw_x + (1 - self.alpha) * self.x_value
        self.y_value = self.alpha * raw_y + (1 - self.alpha) * self.y_value

    def get(self) -> 'tuple[float, float]':
        """
        Get the cached x, y values as a tuple
        """
        return self.x_value, self.y_value

    def update(self, elapsed_time: float):
        self.timer += elapsed_time

        if self.timer >= self.pot_poll_interval:
            self.timer = 0
            self._read()

class ButtonState:
    """
    Handles reading from and debouncing a button and caches the button's toggle state.
    """
    _btn: Pin
    _btn_debounce: float

    _elapsed_time: float

    _toggled_on: bool

    _btn_last_pressed: bool

    def __init__(self, btn_pin: int, btn_debounce: float):
        self._btn = Pin(btn_pin)
        self._btn_debounce = btn_debounce
        self._elapsed_time = 0
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
        """
        self._elapsed_time += elapsed_time

        # get the button's current pressed state
        btn_pressed = self._btn.value()

        # debounce 
        if btn_pressed and not self._btn_last_pressed \
                and self._elapsed_time >= self._btn_debounce:
            self._elapsed_time = 0
            self._toggled_on = not self._toggled_on
        
        self._btn_last_pressed = btn_pressed

class ArmController:
    shoulder: PWM
    elbow: PWM
    wrist: PWM

    def __init__(self, shoulder_pin: int, elbow_pin: int, wrist_pin: int):
        self.shoulder = PWM(Pin(shoulder_pin), freq=50)
        self.elbow = PWM(Pin(elbow_pin), freq=50)
        self.wrist = PWM(Pin(wrist_pin), freq=50)

    def set_arm_angles(self, alpha: float, beta: float) -> None:
        """
        Move the arm to a specific position
        """

        shoulder_duty_cycle = translate(alpha)
        elbow_duty_cycle = translate(beta)

        self.shoulder.duty_u16(shoulder_duty_cycle)
        self.elbow.duty_u16(elbow_duty_cycle)

    def move_wrist(self, pen_down: bool) -> None:
        """
        Set the pen to down or up
        """
        raise NotImplementedError()
