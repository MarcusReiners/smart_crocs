from machine import Pin
import time

# For Pico W, the onboard LED is accessed through the special "LED" constant
led = Pin("LED", Pin.OUT)  # Create LED object using the built-in LED

# Main loop to blink the LED
while True:
    led.value(1)  # Turn on the LED
    time.sleep(0.5)  # Wait for 0.5 seconds
    led.value(0)  # Turn off the LED
    time.sleep(0.5)  # Wait for 0.5 seconds
