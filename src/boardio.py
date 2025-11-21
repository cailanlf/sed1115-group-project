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

    def __init__(self, x_pin: int, y_pin: int, pot_poll_interval: float):
        self.pot_x = ADC(Pin(x_pin))
        self.pot_y = ADC(Pin(y_pin))

        self.x_value = 0
        self.y_value = 0

        self.pot_poll_interval = pot_poll_interval
        self.timer = 0

    def _read(self):
        """
        Read the values from the potentiometers and update the internal state.
        """

        # read the values and convert them to floats
        x = self.pot_x.read_u16() / 65535.0
        y = self.pot_y.read_u16() / 65535.0

        # clamp the x and y to 0.0 - 1.0 (just in case)
        self.x_value = 0. if x < 0. else 1. if x > 1. else x
        self.y_value = 0. if y < 0. else 1. if y > 1. else y

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

    def set_arm_angles(self, alpha: float, beta:float) -> None:
        """
        Move the arm to a specific position
        """
        shoulder_angle = 90 - alpha
        elbow_angle = beta

        shoulder_duty_cycle = translate(shoulder_angle)
        elbow_duty_cycle = translate(elbow_angle)

        self.shoulder.duty_u16(shoulder_duty_cycle)
        self.elbow.duty_u16(elbow_duty_cycle)

    def move_wrist(self, pen_down: bool) -> None:
        """
        Set the pen to down or up
        """
        raise NotImplementedError()
