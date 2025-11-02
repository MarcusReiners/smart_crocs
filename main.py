import machine
import neopixel
import time
import math
from machine import UART, Pin, ADC

# Pin configuration for ESP32-C3 Super Mini
PRESSURE_PIN = 0     # ADC pin for FSR (GPIO0)
LED_PIN = 2          # GPIO pin for LED strip data
MP3_TX_PIN = 6       # TX to MP3 module RX
MP3_RX_PIN = 7       # RX from MP3 module TX
NUM_LEDS = 9         # Number of LEDs in your strip

# Initialize ADC for pressure sensor
pressure_sensor = ADC(Pin(PRESSURE_PIN))
pressure_sensor.atten(ADC.ATTN_11DB)  # Full range 0-3.3V
print("[DEBUG] Pressure sensor initialized on GPIO0")

# Initialize NeoPixel strip
strip = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)
print(f"[DEBUG] NeoPixel initialized with {NUM_LEDS} LEDs on GPIO{LED_PIN}")

# Initialize UART for MP3 module
mp3_uart = UART(1, baudrate=9600, tx=Pin(MP3_TX_PIN), rx=Pin(MP3_RX_PIN))
print(f"[DEBUG] UART initialized for MP3 on TX=GPIO{MP3_TX_PIN}, RX=GPIO{MP3_RX_PIN}")

# Calibration values for FSR
MIN_PRESSURE = 100
MAX_PRESSURE = 3000
PRESSURE_THRESHOLD = 500

# State flags
smoothed_pressure = 0
smoothing_factor = 0.3
triggered = False
audio_played = False
last_trigger_time = 0
debounce_time = 1000  # in milliseconds

# MP3 driver class
class DFPlayer:
    def __init__(self, uart):
        self.uart = uart
        self.volume = 30  # Max volume (0–30)
        time.sleep_ms(1000)
        self.set_volume(self.volume)

    def send_command(self, command, param1=0, param2=0):
        checksum = 0xFFFF - (0xFF + 0x06 + command + 0x00 + param1 + param2) + 1
        checksum &= 0xFFFF

        cmd_bytes = bytearray([
            0x7E, 0xFF, 0x06, command, 0x00, param1, param2,
            (checksum >> 8) & 0xFF, checksum & 0xFF, 0xEF
        ])

        print(f"[DEBUG] Sending UART command: {[hex(b) for b in cmd_bytes]}")
        self.uart.write(cmd_bytes)
        time.sleep_ms(100)

    def play_track(self, track_number):
        self.send_command(0x03, 0x00, track_number)
        print(f"[DEBUG] play_track({track_number}) command sent")

    def set_volume(self, volume):
        volume = max(0, min(30, volume))
        self.send_command(0x06, 0x00, volume)
        self.volume = volume
        print(f"Volume set to {volume}")

    def stop(self):
        self.send_command(0x16)
        print("Stopped")

# Initialize MP3 player
mp3_player = DFPlayer(mp3_uart)

# Read pressure sensor
def read_pressure():
    value = pressure_sensor.read_u16() >> 4
    print(f"[DEBUG] Raw Pressure: {value}")
    return value

# Smoothing
def smooth_reading(new_value, old_value, factor):
    smoothed = int(old_value + factor * (new_value - old_value))
    print(f"[DEBUG] Smoothed Pressure: {smoothed}")
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
    print("[DEBUG] LEDs set to bright blue")

# Handle pressure trigger
def handle_pressure_trigger():
    global triggered, audio_played, last_trigger_time
    current_time = time.ticks_ms()
    if current_time - last_trigger_time < debounce_time:
        return

    if not triggered:
        print("[DEBUG] Triggered: setting LEDs and playing audio")
        set_leds_bright_blue()
        triggered = True
        last_trigger_time = current_time

        if not audio_played:
            mp3_player.play_track(1)
            audio_played = True
    else:
        print("[DEBUG] Already triggered")

# Main loop
def main():
    global smoothed_pressure

    print("ESP32-C3 Pressure-triggered LEDs with MP3 playback")
    print("Press sensor to trigger permanent blue light + one-time audio")
    print("Press Ctrl+C to stop")

    strip.fill((0, 0, 0))
    strip.write()
    mp3_player.set_volume(30)

    print("System ready. Starting main loop...")

    try:
        while True:
            raw_pressure = read_pressure()
            smoothed_pressure = smooth_reading(raw_pressure, smoothed_pressure, smoothing_factor)

            if not triggered:
                if smoothed_pressure >= PRESSURE_THRESHOLD:
                    handle_pressure_trigger()
                else:
                    update_leds_idle_breathing()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
        strip.fill((0, 0, 0))
        strip.write()
        mp3_player.stop()
        print("LEDs turned off, MP3 player stopped")

if __name__ == "__main__":
    main()
