import machine
import neopixel
import time
import math
from machine import Pin, ADC

# Pin configuration for ESP32-C3 Super Mini
PRESSURE_PIN = 0     # ADC pin for FSR (GPIO0)
LED_PIN = 2          # GPIO pin for LED strip data
NUM_LEDS = 9         # Number of LEDs in your strip

# Initialize ADC for pressure sensor
pressure_sensor = ADC(Pin(PRESSURE_PIN))
pressure_sensor.atten(ADC.ATTN_11DB)  # Full range 0-3.3V

# Initialize NeoPixel strip
strip = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# Configuration
PRESSURE_THRESHOLD = 500
smoothed_pressure = 0
smoothing_factor = 0.3
triggered = False

# Read pressure sensor
def read_pressure():
    raw_value = pressure_sensor.read_u16() >> 4
    voltage = (raw_value / 4095) * 3.3  # Convert to voltage
    print(f"[DEBUG] Raw ADC: {raw_value}, Voltage: {voltage:.2f}V")
    return raw_value

# Smooth the pressure reading
def smooth_reading(new_value, old_value, factor):
    smoothed = int(old_value + factor * (new_value - old_value))
    print(f"[DEBUG] Raw: {new_value}, Smoothed: {smoothed}")
    return smoothed

# Breathing LED idle effect
def update_leds_idle_breathing():
    time_factor = time.ticks_ms() / 1000.0
    pulse = (math.sin(time_factor * 1.5) + 1) / 2
    dim_brightness = int(5 + pulse * 10)
    strip.fill((0, 0, dim_brightness))
    strip.write()

# Set LEDs to solid bright blue
def set_leds_bright_blue():
    strip.fill((0, 0, 255))
    strip.write()

# Main loop
def main():
    global smoothed_pressure, triggered

    print("ESP32-C3 Pressure-triggered LEDs")
    print("Press sensor to trigger permanent blue light")
    print("Press Ctrl+C to stop")

    # Turn off LEDs initially
    strip.fill((0, 0, 0))
    strip.write()

    try:
        while True:
            raw_pressure = read_pressure()
            smoothed_pressure = smooth_reading(raw_pressure, smoothed_pressure, smoothing_factor)

            if not triggered:
                if smoothed_pressure >= PRESSURE_THRESHOLD:
                    print(f"[TRIGGER] Pressure threshold reached! Smoothed: {smoothed_pressure} >= {PRESSURE_THRESHOLD}")
                    set_leds_bright_blue()
                    triggered = True
                else:
                    update_leds_idle_breathing()
                    
                # Show pressure status every 20 cycles (~1 second)
                if time.ticks_ms() % 1000 < 50:
                    print(f"[STATUS] Threshold: {PRESSURE_THRESHOLD}, Current: {smoothed_pressure}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
        strip.fill((0, 0, 0))
        strip.write()
        print("LEDs turned off")

if __name__ == "__main__":
    main()