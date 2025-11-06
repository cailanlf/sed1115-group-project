from machine import Pin, ADC
from time import time_ns

# Configuration
pot_pin_x = ADC(Pin(27))
pot_pin_y = ADC(Pin(26))

btn_pin = Pin(12)
btn_debounce = 50.

def main():
    poll_timer = 0
    btn_debounce_timer = 0
    pen_state = False

    btn_last_state = 0

    start = time_ns()
    while True:
        # update time 
        end = time_ns()
        elapsed = (end - start) / 1_000_000
        start = end

        poll_timer += elapsed
        btn_debounce_timer += elapsed
        
        # read and print the potentiometer values every 50 ms
        if poll_timer > 100:
            poll_timer = 0
            x = pot_pin_x.read_u16()
            y = pot_pin_y.read_u16()
            print(f"\r{x: <5}, {y: <5} pen={'DOWN' if pen_state else 'UP  '}", end="")

        # check if button is pressed and debounce appropriately
        if btn_pin.value() and not btn_last_state and btn_debounce_timer > btn_debounce:
            pen_state = not pen_state
            btn_debounce_timer = 0

        btn_last_state = btn_pin.value()

if __name__ == "__main__":
    main()

