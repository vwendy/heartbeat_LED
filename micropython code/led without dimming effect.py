"""
WS2812 RGB LED Ring Light Breathing with Heart Rate Detection & OLED Display
Raspberry Pi Pico Microcontroller

Original Author: Joshua Hrisko, Maker Portal LLC (c) 2021
Based on: https://github.com/raspberrypi/pico-micropython-examples

This module drives a WS2812 RGB LED ring and displays real-time PPG (photoplethysmography)
sensor data on an OLED display while modulating LED brightness based on detected heartbeat patterns.

Key Features:
- WS2812 addressable LED control via PIO
- PPG signal acquisition and processing
- Heart rate detection via derivative-based peak detection
- Real-time waveform visualization on 128x64 OLED display
- Color-coded LED feedback based on heart rate
"""

import array
import math
import utime
from machine import Pin, ADC, I2C
import rp2
from ssd1306 import SSD1306_I2C


# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

# LED Ring Configuration
LED_COUNT = 11              # Number of LEDs in the ring light
LED_PIN_NUM = 2             # GPIO pin connected to WS2812 data line
LED_BRIGHTNESS = 1.0        # Global brightness multiplier (0.1 = dim, 1.0 = bright)
LED_BRIGHTNESS_VAR = 0.075  # Additional brightness variation factor (unused in current code)
SPEED = 255                 # Speed parameter for breathing animation (255 = fastest)

# PPG Sensor Configuration
PPG_ADC_PIN = 28            # ADC pin connected to the pulse sensor

# OLED Display Configuration
OLED_RES_X = 128            # OLED display width in pixels
OLED_RES_Y = 64             # OLED display height in pixels
OLED_I2C_ADDR = 0           # I2C bus 0
OLED_SDA_PIN = 0            # GPIO pin for SDA (data line)
OLED_SCL_PIN = 1            # GPIO pin for SCL (clock line)
OLED_I2C_FREQ = 400000      # I2C frequency in Hz

# Heart Rate Detection Parameters
WINDOW_SIZE = 20            # Moving average window size (samples)
MAX_MIN_WINDOW_SIZE = OLED_RES_X  # Window for min/max threshold (128 samples = display width)
PEAK_LIST_SIZE = 5          # Number of peaks to track for BPM calculation
TIMING_THRESHOLD = 300      # Minimum time between peaks in milliseconds
PEAK_THRESHOLD_PERCENT = 0.7  # Threshold is 70% of signal range above minimum

# Brightness Scaling Constants
ADC_MAX_VALUE = 65535       # Maximum value for 16-bit ADC (2^16 - 1)
BRIGHTNESS_DIVISOR = 6553600  # Divisor for linear brightness scaling


# ============================================================================
# LED HARDWARE INITIALIZATION
# ============================================================================

def initialize_ws2812_state_machine():
    """
    Initialize the WS2812 LED protocol state machine using PIO.
    
    WS2812 LEDs use a specific timing protocol:
    - T1: High pulse time for bit '1' (2 cycles)
    - T2: Low pulse time for bit '1' (5 cycles)
    - T3: High pulse time for bit '0' (3 cycles)
    
    Returns:
        StateMachine: Configured PIO state machine for WS2812 control
    """
    @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT,
                 autopull=True, pull_thresh=24)
    def ws2812_protocol():
        """WS2812 protocol implementation in PIO assembly."""
        T1 = 2  # Cycles for bit '1' high time
        T2 = 5  # Cycles for bit '1' low time
        T3 = 3  # Cycles for bit '0' high time
        
        wrap_target()
        label("bitloop")
        out(x, 1)               .side(0)    [T3 - 1]
        jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
        jmp("bitloop")          .side(1)    [T2 - 1]
        label("do_zero")
        nop()                   .side(0)    [T2 - 1]
        wrap()
    
    # Create and activate the state machine at 8MHz frequency
    sm = rp2.StateMachine(0, ws2812_protocol, freq=8_000_000, 
                          sideset_base=Pin(LED_PIN_NUM))
    sm.active(1)
    return sm


# ============================================================================
# OLED DISPLAY INITIALIZATION
# ============================================================================

def initialize_oled_display():
    """
    Initialize the SSD1306 OLED display via I2C.
    
    Returns:
        tuple: (I2C object, SSD1306_I2C display object)
    """
    # Initialize I2C bus 0 with specified SDA and SCL pins
    i2c = I2C(OLED_I2C_ADDR, sda=Pin(OLED_SDA_PIN), scl=Pin(OLED_SCL_PIN), 
              freq=OLED_I2C_FREQ)
    
    # Create OLED display object
    oled = SSD1306_I2C(OLED_RES_X, OLED_RES_Y, i2c)
    
    # Display startup message
    oled.text("Heart Monitor", 0, 0)
    oled.text("Initializing...", 0, 10)
    oled.show()
    
    return i2c, oled


# Initialize hardware
pulse_sensor = ADC(PPG_ADC_PIN)
state_machine = initialize_ws2812_state_machine()
i2c_bus, oled_display = initialize_oled_display()

# LED color buffer for storing color values
led_color_buffer = array.array("I", [0 for _ in range(LED_COUNT)])


# ============================================================================
# LED COLOR CONTROL FUNCTIONS
# ============================================================================

def pixels_set(led_index, color):
    """
    Set the color of a single LED without updating the display.
    
    Args:
        led_index (int): Index of the LED to set (0 to LED_COUNT-1)
        color (tuple): RGB color as (red, green, blue), each 0-255
    """
    r, g, b = color[0], color[1], color[2]
    # Convert RGB to 24-bit format: GRB order (WS2812 uses GRB byte order)
    led_color_buffer[led_index] = (g << 16) + (r << 8) + b


def pixels_show(brightness_input=LED_BRIGHTNESS):
    """
    Apply brightness adjustment and send colors to all LEDs.
    
    Extracts the R, G, B components from each LED's color value, applies
    brightness scaling, and sends the dimmed colors to the LED state machine.
    
    Args:
        brightness_input (float): Brightness multiplier (0.0 to 1.0+)
    """
    dimmer_buffer = array.array("I", [0 for _ in range(LED_COUNT)])
    
    for led_idx, color_24bit in enumerate(led_color_buffer):
        # Extract 8-bit color components from 24-bit value
        red = int(((color_24bit >> 8) & 0xFF) * brightness_input)
        green = int(((color_24bit >> 16) & 0xFF) * brightness_input)
        blue = int((color_24bit & 0xFF) * brightness_input)
        
        # Recombine with brightness applied
        dimmer_buffer[led_idx] = (green << 16) + (red << 8) + blue
    
    # Send to state machine for display
    state_machine.put(dimmer_buffer, 8)


def set_all_leds(color, brightness=LED_BRIGHTNESS):
    """
    Set all LEDs to the same color with specified brightness.
    
    Args:
        color (tuple): RGB color as (red, green, blue), each 0-255
        brightness (float): Brightness multiplier (0.0 to 1.0+)
    """
    for i in range(LED_COUNT):
        pixels_set(i, color)
    pixels_show(brightness)


def breathing_led(color, pulse_value):
    """
    Display LEDs in specified color with brightness modulated by pulse.
    
    The brightness is calculated as a linear function of the raw PPG sensor
    value, creating a "breathing" effect that pulses with the user's heartbeat.
    
    Args:
        color (tuple): RGB color as (red, green, blue), each 0-255
        pulse_value (int): Raw 16-bit PPG sensor reading (0-65535)
    """
    # Set all LEDs to the specified color
    for led_idx in range(LED_COUNT):
        pixels_set(led_idx, color)
    
    # Calculate brightness based on pulse (linear scaling)
    # Formula: normalize 16-bit sensor value to 0.0-1.0 range
    # brightness_divisor = 6553600 was calibrated for this sensor
    brightness = pulse_value / BRIGHTNESS_DIVISOR
    
    # Clamp brightness to valid range
    brightness = max(0.0, min(1.0, brightness))
    
    # Update LED display with pulse-modulated brightness
    pixels_show(brightness)


# ============================================================================
# COLOR DEFINITIONS
# ============================================================================

RED = (255, 0, 0)
ORANGE = (255, 125, 0)
YELLOW = (255, 255, 0)
SPRING_GREEN = (125, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
VIOLET = (125, 0, 255)
MAGENTA = (255, 0, 255)
PINK = (175, 0, 75)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def map_value(x, in_min, in_max, out_min, out_max):
    """
    Map a value from one range to another (linear interpolation).
    
    Args:
        x (float): Input value to map
        in_min (float): Minimum of input range
        in_max (float): Maximum of input range
        out_min (float): Minimum of output range
        out_max (float): Maximum of output range
        
    Returns:
        float: Mapped value in the output range
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# ============================================================================
# HEART RATE DETECTION - SIGNAL PROCESSOR CLASS
# ============================================================================

class PPGSignalProcessor:
    """
    Processes PPG (photoplethysmography) sensor data and detects heartbeats.
    
    This class implements a derivative-based peak detection algorithm with
    adaptive thresholding to identify heartbeats from raw PPG data. It also
    maintains a history of sensor values for visualization on the OLED display.
    """
    
    def __init__(self):
        """Initialize all data structures for signal processing."""
        # Moving average for noise reduction
        self.moving_avg_window = [0] * WINDOW_SIZE
        self.moving_avg_idx = 0
        self.moving_average = 0
        self.prev_moving_average = 0
        
        # Derivative tracking for peak detection
        self.current_diff = 0
        self.prev_diff = 0
        self.diff_product = 0  # Product of consecutive derivatives
        
        # Adaptive threshold computation (also used for OLED visualization)
        self.max_min_window = [0] * MAX_MIN_WINDOW_SIZE
        self.max_min_idx = 0
        self.adaptive_threshold = 0
        
        # Peak tracking for BPM calculation
        self.peak_timestamps = [0] * PEAK_LIST_SIZE
        self.peak_idx = 0
        self.beat_count = 0
        self.last_peak_time = utime.ticks_ms()
        self.current_bpm = 0
        
        # OLED update counter
        self.update_count = 0
    
    def update(self, raw_sensor_value):
        """
        Process a new PPG sensor reading and update all calculated metrics.
        
        Args:
            raw_sensor_value (int): 16-bit raw value from PPG sensor
            
        Returns:
            dict: Contains moving_average, threshold, peak_detected, and bpm
        """
        # Step 1: Update moving average filter
        self._update_moving_average(raw_sensor_value)
        
        # Step 2: Calculate derivative for peak detection
        self._update_derivatives()
        
        # Step 3: Compute adaptive threshold
        self._update_adaptive_threshold(raw_sensor_value)
        
        # Step 4: Detect peaks and update BPM
        peak_detected = self._detect_peak()
        
        return {
            'moving_average': self.moving_average,
            'threshold': self.adaptive_threshold,
            'peak_detected': peak_detected,
            'bpm': self.current_bpm,
            'should_update_oled': self._should_update_oled()
        }
    
    def _update_moving_average(self, raw_value):
        """Update the moving average window with new sensor value."""
        self.moving_avg_window[self.moving_avg_idx] = raw_value
        self.moving_avg_idx = (self.moving_avg_idx + 1) % WINDOW_SIZE
        
        # Compute average (integer division for efficiency)
        self.moving_average = sum(self.moving_avg_window) // WINDOW_SIZE
    
    def _update_derivatives(self):
        """
        Calculate the derivative (rate of change) of the smoothed signal.
        
        The product of consecutive derivatives helps identify peaks:
        - When derivative changes from positive to negative: peak detected
        - Product < 0 indicates sign change (more accurate than <= 0)
        """
        self.current_diff = self.moving_average - self.prev_moving_average
        self.diff_product = self.current_diff * self.prev_diff
        
        self.prev_moving_average = self.moving_average
        self.prev_diff = self.current_diff
    
    def _update_adaptive_threshold(self, raw_value):
        """
        Compute adaptive threshold based on recent signal range.
        
        Threshold = 70% of signal range + minimum value
        
        This adapts to changing signal characteristics and prevents
        false peaks from sensor noise. The window is also used for
        OLED waveform visualization.
        """
        self.max_min_window[self.max_min_idx] = raw_value
        self.max_min_idx = (self.max_min_idx + 1) % MAX_MIN_WINDOW_SIZE
        
        signal_min = min(self.max_min_window)
        signal_range = max(self.max_min_window) - signal_min
        
        self.adaptive_threshold = (signal_range * PEAK_THRESHOLD_PERCENT) + signal_min
    
    def _detect_peak(self):
        """
        Detect a heartbeat peak based on three criteria:
        1. Derivative sign change (product < 0)
        2. Signal above adaptive threshold
        3. Sufficient time since last peak (refractory period)
        
        Returns:
            bool: True if a new peak was detected, False otherwise
        """
        current_time = utime.ticks_ms()
        time_since_last_peak = utime.ticks_diff(current_time, self.last_peak_time)
        
        # Check all three peak detection criteria
        is_peak_crossing = (self.diff_product < 0)  # Derivative sign change
        is_above_threshold = (self.moving_average >= self.adaptive_threshold)
        is_sufficient_interval = (time_since_last_peak >= TIMING_THRESHOLD)
        
        if is_peak_crossing and is_above_threshold and is_sufficient_interval:
            # Peak detected - update peak tracking
            self.beat_count += 1
            self.last_peak_time = current_time
            self.peak_timestamps[self.peak_idx] = current_time
            self.peak_idx = (self.peak_idx + 1) % PEAK_LIST_SIZE
            
            # Calculate BPM once we have enough peaks
            if self.beat_count >= 10:  # Require at least 10 beats for stable BPM
                self._calculate_bpm()
            
            return True
        
        return False
    
    def _calculate_bpm(self):
        """
        Calculate beats per minute from the last PEAK_LIST_SIZE peaks.
        
        Formula: (number_of_peaks * 60 seconds) / time_span_in_seconds
        The result is multiplied by 1000 to convert from milliseconds to seconds.
        """
        time_span_ms = utime.ticks_diff(
            max(self.peak_timestamps),
            min(self.peak_timestamps)
        )
        
        # Avoid division by zero
        if time_span_ms > 0:
            self.current_bpm = (PEAK_LIST_SIZE * 60000) / time_span_ms
    
    def _should_update_oled(self):
        """
        Check if OLED display should be updated.
        
        Returns:
            bool: True when max_min_window is completely filled (display cycle complete)
        """
        self.update_count += 1
        if self.update_count >= MAX_MIN_WINDOW_SIZE:
            self.update_count = 0
            return True
        return False
    
    def get_waveform_data(self):
        """
        Get the current waveform data for OLED visualization.
        
        Returns:
            list: Raw sensor values from max_min_window
        """
        return self.max_min_window


# ============================================================================
# OLED DISPLAY FUNCTIONS
# ============================================================================

def plot_ppg_waveform(oled, waveform_data, resolution_x, resolution_y):
    """
    Plot the PPG waveform on the OLED display.
    
    Normalizes the waveform data to fit within the display resolution
    and draws it as a series of dots.
    
    Args:
        oled (SSD1306_I2C): OLED display object
        waveform_data (list): Raw sensor values to plot
        resolution_x (int): Display width in pixels
        resolution_y (int): Display height in pixels
    """
    oled.fill(0)  # Clear the display
    
    # Plot each point in the waveform
    for x_pos in range(resolution_x):
        if x_pos < len(waveform_data):
            # Normalize sensor value (0-65535) to display pixel range
            normalized = (waveform_data[x_pos] / ADC_MAX_VALUE) * (resolution_y - 1)
            
            # Invert Y coordinate (top of display is pixel 0)
            y_pos = resolution_y - int(normalized)
            
            # Draw a dot at this position
            oled.text('.', x_pos, y_pos)
    
    oled.show()  # Render the updates to the screen


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def get_led_color_for_bpm(bpm):
    """
    Return LED color based on current heart rate.
    
    Args:
        bpm (float): Current heart rate in beats per minute
        
    Returns:
        tuple: RGB color tuple
    """
    if bpm < 60:
        return BLUE      # Slow heart rate (bradycardia warning)
    elif bpm > 100:
        return RED       # Elevated heart rate (tachycardia warning)
    else:
        return GREEN     # Normal heart rate


def main():
    """Main event loop: continuously read sensor data and update LEDs/OLED."""
    print("\n" + "="*60)
    print("Heart Rate Monitor - Initialized")
    print("="*60)
    print(f"LED Count: {LED_COUNT}")
    print(f"OLED Resolution: {OLED_RES_X}x{OLED_RES_Y}")
    print(f"Monitoring PPG sensor on ADC pin {PPG_ADC_PIN}")
    print("Waiting for heartbeat detection...\n")
    
    # Initialize signal processor
    ppg_processor = PPGSignalProcessor()
    
    # Main loop - runs indefinitely
    try:
        while True:
            # Read raw PPG sensor value (16-bit)
            raw_ppg_value = pulse_sensor.read_u16()
            
            # Process the sensor reading
            result = ppg_processor.update(raw_ppg_value)
            
            # Get the color based on current BPM
            led_color = get_led_color_for_bpm(ppg_processor.current_bpm)
            
            # Update LED display with pulse-modulated brightness
            breathing_led(led_color, raw_ppg_value)
            
            # Update OLED display once per full waveform cycle
            if result['should_update_oled']:
                plot_ppg_waveform(oled_display, ppg_processor.get_waveform_data(),
                                OLED_RES_X, OLED_RES_Y)
            
            # Print BPM when a new peak is detected
            if result['peak_detected']:
                print(f"♥ Peak detected! BPM: {int(ppg_processor.current_bpm)}")
    
    except KeyboardInterrupt:
        # Graceful shutdown
        print("\n\n" + "="*60)
        print("Monitor stopped by user")
        print("="*60)
        set_all_leds(BLACK, 0)  # Turn off all LEDs
        oled_display.fill(0)
        oled_display.text("Monitor stopped", 0, 30)
        oled_display.show()


if __name__ == "__main__":
    main()
