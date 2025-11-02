import machine
import neopixel
import time
import math

# Pin configuration
PRESSURE_PIN = 26    # ADC0 pin for FSR
LED_PIN = 2          # GPIO pin for LED strip data
SPEAKER_PIN = 20     # GPIO pin for speaker
NUM_LEDS = 9        # Number of LEDs in your strip

# Initialize ADC for pressure sensor
pressure_sensor = machine.ADC(PRESSURE_PIN)

# Initialize NeoPixel strip
strip = neopixel.NeoPixel(machine.Pin(LED_PIN), NUM_LEDS)

# Initialize PWM for speaker
speaker = machine.PWM(machine.Pin(SPEAKER_PIN))

# Calibration values for FSR (adjust based on testing)
MIN_PRESSURE = 100       # Minimum pressure reading (light touch)
MAX_PRESSURE = 5000      # Maximum pressure reading (strong press)
PRESSURE_THRESHOLD = 3000  # Minimum pressure to activate full effects

# Variables for smooth operation
last_brightness = -1
smoothed_pressure = 0
smoothing_factor = 0.3  # Lower = more smoothing

def read_pressure():
    return pressure_sensor.read_u16() >> 4

def smooth_reading(new_value, old_value, factor):
    return int(old_value + factor * (new_value - old_value))

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def constrain(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def hsv_to_rgb(h, s, v):
    h = h / 360.0
    s = s / 100.0
    v = v / 100.0

    i = int(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)

    rgb_map = [
        (v, t, p), (q, v, p), (p, v, t),
        (p, q, v), (t, p, v), (v, p, q)
    ]

    r, g, b = rgb_map[i % 6]
    return (int(r * 255), int(g * 255), int(b * 255))

# Speaker functions
def play_tone(frequency, duration_ms, volume=50):
    """Play a tone at given frequency for specified duration"""
    if frequency > 0:
        speaker.freq(frequency)
        # Volume control: 0-100, converted to duty cycle (0-65535)
        duty = int((volume / 100.0) * 32768)  # 50% max duty cycle for safety
        speaker.duty_u16(duty)
        time.sleep_ms(duration_ms)
    speaker.duty_u16(0)  # Turn off

def play_beep():
    """Simple beep sound"""
    play_tone(1000, 100)

def play_startup_sound():
    """Play a startup melody"""
    notes = [262, 330, 392, 523]  # C, E, G, C (one octave higher)
    for note in notes:
        play_tone(note, 200, 30)
        time.sleep_ms(50)

def play_pressure_tone(pressure_value):
    """Play tone based on pressure - higher pressure = higher pitch"""
    if pressure_value > PRESSURE_THRESHOLD:
        frequency = map_value(pressure_value, PRESSURE_THRESHOLD, MAX_PRESSURE, 200, 2000)
        frequency = constrain(frequency, 200, 2000)
        volume = map_value(pressure_value, PRESSURE_THRESHOLD, MAX_PRESSURE, 10, 60)
        volume = constrain(volume, 10, 60)
        play_tone(frequency, 50, volume)

def test_speaker():
    """Test speaker with various tones and patterns"""
    print("=== Speaker Test ===")
    print("Testing basic tones...")
    
    # Test basic frequencies
    frequencies = [262, 330, 392, 523, 659, 784, 1047]  # C major scale
    note_names = ['C', 'E', 'G', 'C', 'E', 'G', 'C']
    
    for freq, name in zip(frequencies, note_names):
        print(f"Playing {name} ({freq} Hz)")
        play_tone(freq, 300, 40)
        time.sleep_ms(100)
    
    print("Testing volume levels...")
    for volume in [10, 25, 50, 75]:
        print(f"Volume: {volume}%")
        play_tone(440, 200, volume)  # A note
        time.sleep_ms(200)
    
    print("Testing siren sound...")
    for i in range(20):
        freq = 300 + (i % 10) * 50
        play_tone(freq, 50, 30)
    
    print("Testing beep function...")
    for i in range(3):
        play_beep()
        time.sleep_ms(300)
    
    print("Playing startup sound...")
    play_startup_sound()
    
    print("Speaker test complete!")

def stop_speaker():
    """Safely stop speaker"""
    speaker.duty_u16(0)
    speaker.deinit()

def update_leds_blue(brightness):
    if brightness == 0:
        strip.fill((0, 0, 0))
    else:
        strip.fill((0, 0, brightness))
    strip.write()

def update_leds_white(brightness):
    if brightness == 0:
        strip.fill((0, 0, 0))
    else:
        strip.fill((brightness, brightness, brightness))
    strip.write()

def update_leds_rainbow(brightness):
    if brightness == 0:
        strip.fill((0, 0, 0))
    else:
        for i in range(NUM_LEDS):
            hue = (i * 360 // NUM_LEDS + brightness) % 360
            saturation = 100
            value = (brightness * 100) // 255
            r, g, b = hsv_to_rgb(hue, saturation, value)
            strip[i] = (r, g, b)
    strip.write()

def update_leds_progress(brightness):
    num_lit = (brightness * NUM_LEDS) // 255
    for i in range(NUM_LEDS):
        if i < num_lit:
            red = map_value(brightness, 0, 255, 0, 255)
            green = map_value(brightness, 0, 255, 255, 0)
            strip[i] = (red, green, 0)
        else:
            strip[i] = (0, 0, 0)
    strip.write()

def update_leds_breathing(brightness):
    if brightness == 0:
        strip.fill((0, 0, 0))
    else:
        time_factor = time.ticks_ms() / 1000.0
        pulse = (math.sin(time_factor * 2) + 1) / 2
        current_brightness = int(brightness * pulse)
        strip.fill((0, current_brightness // 2, current_brightness))
    strip.write()

def update_leds_fire(brightness):
    if brightness == 0:
        strip.fill((0, 0, 0))
    else:
        for i in range(NUM_LEDS):
            flicker = (time.ticks_ms() + i * 123) % 100
            flicker_factor = 0.7 + (flicker / 100.0) * 0.3
            red = int(brightness * flicker_factor)
            green = int((brightness * flicker_factor) // 3)
            blue = 0
            strip[i] = (red, green, blue)
    strip.write()

def update_leds_idle_breathing():
    """Idle breathing effect with dim blue light"""
    time_factor = time.ticks_ms() / 1000.0
    pulse = (math.sin(time_factor * 1.5) + 1) / 2
    dim_brightness = int(5 + pulse * 10)  # Range: 5–15
    strip.fill((0, 0, dim_brightness))
    strip.write()

def test_led_strip():
    print("Testing LED strip...")
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)]
    color_names = ["Red", "Green", "Blue", "White"]
    for color, name in zip(colors, color_names):
        print(f"Testing {name}...")
        strip.fill(color)
        strip.write()
        time.sleep(1)
    print("Turning off...")
    strip.fill((0, 0, 0))
    strip.write()
    print("LED test complete!")

def calibrate_sensor():
    print("=== FSR Calibration ===")
    print("Press Ctrl+C to stop calibration")
    print("Apply different pressure levels and note the readings:")
    min_reading = 4095
    max_reading = 0
    try:
        while True:
            reading = read_pressure()
            min_reading = min(min_reading, reading)
            max_reading = max(max_reading, reading)
            print(f"Current: {reading:4d} | Min: {min_reading:4d} | Max: {max_reading:4d}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\nCalibration complete!")
        print(f"Suggested MIN_PRESSURE: {min_reading + 50}")
        print(f"Suggested MAX_PRESSURE: {max_reading}")
        print(f"Suggested THRESHOLD: {min_reading + 100}")

def run_tests():
    """Run all component tests"""
    print("=== Component Testing Mode ===")
    print("1. Testing LED Strip")
    test_led_strip()
    time.sleep(1)
    
    print("\n2. Testing Speaker")
    test_speaker()
    time.sleep(1)
    
    print("\n3. Testing Pressure Sensor")
    print("Apply pressure to test sensor readings...")
    for i in range(50):  # Test for 5 seconds
        pressure = read_pressure()
        print(f"Pressure reading: {pressure}")
        time.sleep(0.1)
    
    print("\nAll tests complete!")

def main():
    global last_brightness, smoothed_pressure
    print("Pressure-controlled LED strip with sound starting...")
    print(f"Using {NUM_LEDS} LEDs on GPIO{LED_PIN}")
    print(f"Speaker on GPIO{SPEAKER_PIN}")
    print("Apply pressure to sensor to control LED brightness and sound")
    print("Press Ctrl+C to stop")
    
    # Play startup sound
    play_startup_sound()
    
    strip.fill((0, 0, 0))
    strip.write()
    
    try:
        while True:
            raw_pressure = read_pressure()
            smoothed_pressure = smooth_reading(raw_pressure, smoothed_pressure, smoothing_factor)

            if smoothed_pressure < PRESSURE_THRESHOLD:
                update_leds_idle_breathing()
                last_brightness = -1
            else:
                brightness = map_value(smoothed_pressure, PRESSURE_THRESHOLD, MAX_PRESSURE, 0, 255)
                brightness = constrain(brightness, 0, 255)

                if abs(brightness - last_brightness) > 2:
                    update_leds_blue(brightness)  # Choose your active effect here
                    # update_leds_white(brightness)
                    # update_leds_rainbow(brightness)
                    # update_leds_progress(brightness)
                    # update_leds_breathing(brightness)
                    # update_leds_fire(brightness)
                    
                    # Play pressure-based tone
                    play_pressure_tone(smoothed_pressure)

                    last_brightness = brightness
                    print(f"Raw: {raw_pressure:4d} | Smooth: {smoothed_pressure:4d} | Brightness: {brightness:3d}")

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
        strip.fill((0, 0, 0))
        strip.write()
        stop_speaker()
        print("LEDs turned off, speaker stopped")

# Menu system
def show_menu():
    print("\n=== Pressure LED & Speaker Control ===")
    print("1. Run main program")
    print("2. Test all components")
    print("3. Test speaker only")
    print("4. Test LED strip only")
    print("5. Calibrate pressure sensor")
    print("6. Exit")
    return input("Choose option (1-6): ")

# Run the program
if __name__ == "__main__":
    try:
        while True:
            choice = show_menu()
            
            if choice == '1':
                main()
            elif choice == '2':
                run_tests()
            elif choice == '3':
                test_speaker()
            elif choice == '4':
                test_led_strip()
            elif choice == '5':
                calibrate_sensor()
            elif choice == '6':
                print("Goodbye!")
                stop_speaker()
                break
            else:
                print("Invalid choice, please try again.")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted")
        stop_speaker()
    except Exception as e:
        print(f"An error occurred: {e}")
        stop_speaker()