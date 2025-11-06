from machine import Pin, ADC
from time import time_ns

# Configuration
pot_pin_x = ADC(Pin(27))
pot_pin_y = ADC(Pin(26))

def main():
    poll_timer = 0

    start = time_ns()
    while True:
        # update time 
        end = time_ns()
        elapsed = (end - start) / 1_000_000
        start = end

        poll_timer += elapsed
        
        # read and print the potentiometer values every 50 ms
        if poll_timer > 100:
            poll_timer = 0
            x = pot_pin_x.read_u16()
            y = pot_pin_y.read_u16()
            print(f"\r{x: <5}, {y: <5}", end="")

if __name__ == "__main__":
    main()

