"""
WS2812 RGB LED Ring Light Breathing with Heart Rate Detection
Raspberry Pi Pico Microcontroller

Original Author: Joshua Hrisko, Maker Portal LLC (c) 2021
Based on: https://github.com/raspberrypi/pico-micropython-examples

This module drives a WS2812 RGB LED ring and detects heart rate via PPG
(photoplethysmography) sensor data, displaying real-time waveforms on an
OLED display with color-coded LED feedback.

Key Features:
- WS2812 addressable LED control via PIO
- PPG signal acquisition and processing
- Heart rate detection via derivative-based peak detection
- Real-time waveform visualization on 128x64 OLED display
- Color-coded LED feedback based on heart rate
"""

import array
import utime
from machine import ADC, Pin, I2C
import rp2
from ssd1306 import SSD1306_I2C


# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

# LED Ring Configuration
LED_COUNT = 11              # Number of LEDs in the ring light
LED_PIN_NUM = 2             # GPIO pin connected to WS2812 data line
LED_BRIGHTNESS = 1.0        # Global brightness multiplier (0.1 = dim, 1.0 = bright)
LED_BRIGHTNESS_VAR = 0.075  # Additional brightness variation (currently unused)
LED_SPEED = 255             # Speed parameter (255 = fastest animation)

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
MAX_MIN_WINDOW_SIZE = OLED_RES_X  # Window for min/max threshold (128 samples)
PEAK_LIST_SIZE = 5          # Number of peaks to track for BPM calculation
TIMING_THRESHOLD = 300      # Minimum time between peaks (milliseconds)
PEAK_THRESHOLD_PERCENT = 0.6  # Threshold is 70% of signal range above minimum
BEAT_COUNT_MIN = 10         # Minimum beats before calculating stable BPM

# ADC Configuration
ADC_MAX_VALUE = 65535       # Maximum value for 16-bit ADC (2^16 - 1)
BRIGHTNESS_DIVISOR = 65536 * 25  # Divisor for linear brightness scaling


# ============================================================================
# LED HARDWARE INITIALIZATION
# ============================================================================

def initialize_ws2812_state_machine():
    """
    Initialize the WS2812 LED protocol state machine using PIO (Programmable I/O).
    
    WS2812 LEDs use a specific timing protocol:
    - T1: High pulse time for bit '1' (2 cycles)
    - T2: Low pulse time for bit '1' (5 cycles)
    - T3: High pulse time for bit '0' (3 cycles)
    
    This function creates the PIO program and activates the state machine
    to control the LED data transmission at 8MHz frequency.
    
    Returns:
        StateMachine: Configured PIO state machine for WS2812 control
    """
    @rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, 
                 out_shiftdir=rp2.PIO.SHIFT_LEFT,
                 autopull=True, pull_thresh=24)
    def ws2812_protocol():
        """
        WS2812 protocol implementation in PIO assembly.
        
        This assembly code handles the bit-level timing required by WS2812 LEDs:
        - Checks each output bit
        - Generates appropriate high/low pulses based on bit value
        - Automatically pulls new data when the shift register is empty
        """
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
    state_machine = rp2.StateMachine(0, ws2812_protocol, 
                                      freq=8_000_000, 
                                      sideset_base=Pin(LED_PIN_NUM))
    state_machine.active(1)
    return state_machine


# ============================================================================
# OLED DISPLAY INITIALIZATION
# ============================================================================

def initialize_oled_display():
    """
    Initialize the SSD1306 OLED display via I2C.
    
    Sets up I2C communication with the OLED display and displays
    an initialization message.
    
    Returns:
        tuple: (I2C object, SSD1306_I2C display object)
    """
    # Initialize I2C bus 0 with specified pins
    i2c = I2C(OLED_I2C_ADDR, 
              sda=Pin(OLED_SDA_PIN), 
              scl=Pin(OLED_SCL_PIN), 
              freq=OLED_I2C_FREQ)
    
    # Create OLED display object
    oled = SSD1306_I2C(OLED_RES_X, OLED_RES_Y, i2c)
    
    # Display startup message
    oled.text("Heart Monitor", 0, 0)
    oled.show()
    
    return i2c, oled


# ============================================================================
# HARDWARE INITIALIZATION
# ============================================================================

# Initialize PPG sensor (ADC on pin 28)
pulse_sensor = ADC(PPG_ADC_PIN)

# Initialize WS2812 LED state machine
state_machine = initialize_ws2812_state_machine()

# Initialize OLED display
i2c_bus, oled_display = initialize_oled_display()

# LED color buffer for storing 24-bit GRB color values
led_color_buffer = array.array("I", [0 for _ in range(LED_COUNT)])


# ============================================================================
# LED COLOR CONTROL FUNCTIONS
# ============================================================================

def pixels_set(led_index, color):
    """
    Set the color of a single LED in the buffer (does not update display).
    
    Converts RGB tuples to 24-bit GRB format required by WS2812 LEDs.
    
    Args:
        led_index (int): Index of the LED to set (0 to LED_COUNT-1)
        color (tuple): RGB color as (red, green, blue), each 0-255
        
    Example:
        pixels_set(0, (255, 0, 0))  # Set LED 0 to red
    """
    red, green, blue = color[0], color[1], color[2]
    # WS2812 uses GRB byte order (not RGB)
    led_color_buffer[led_index] = (green << 16) + (red << 8) + blue


def pixels_show(brightness_input=LED_BRIGHTNESS):
    """
    Apply brightness adjustment and send colors to all LEDs.
    
    Extracts RGB components from each LED's color value, applies
    brightness scaling (dimming), and sends the dimmed colors to
    the LED state machine for display.
    
    Args:
        brightness_input (float): Brightness multiplier (0.0 to 1.0+)
        
    Example:
        pixels_show(0.5)  # Display at 50% brightness
    """
    dimmer_buffer = array.array("I", [0 for _ in range(LED_COUNT)])
    
    for led_idx, color_24bit in enumerate(led_color_buffer):
        # Extract 8-bit color components from 24-bit GRB value
        red = int(((color_24bit >> 8) & 0xFF) * brightness_input)
        green = int(((color_24bit >> 16) & 0xFF) * brightness_input)
        blue = int((color_24bit & 0xFF) * brightness_input)
        
        # Recombine with brightness applied, maintaining GRB order
        dimmer_buffer[led_idx] = (green << 16) + (red << 8) + blue
    
    # Send dimmed colors to state machine for display
    state_machine.put(dimmer_buffer, 8)


def set_all_leds(color, brightness=LED_BRIGHTNESS):
    """
    Set all LEDs to the same color with specified brightness.
    
    Args:
        color (tuple): RGB color as (red, green, blue), each 0-255
        brightness (float): Brightness multiplier (0.0 to 1.0+)
        
    Example:
        set_all_leds((0, 255, 0), 0.8)  # Set all LEDs to green at 80% brightness
    """
    for i in range(LED_COUNT):
        pixels_set(i, color)
    pixels_show(brightness)


def breathing_led(color, pulse_value):
    """
    Display LEDs in specified color with brightness modulated by PPG pulse.
    
    Creates a "breathing" effect where LED brightness pulses with the user's
    heartbeat. Brightness is calculated as a linear function of the raw PPG
    sensor value.
    
    Args:
        color (tuple): RGB color as (red, green, blue), each 0-255
        pulse_value (int): Raw 16-bit PPG sensor reading (0-65535)
        
    Note:
        The BRIGHTNESS_DIVISOR constant should be calibrated for your specific
        sensor to achieve appropriate brightness response.
    """
    # Set all LEDs to the specified color
    for led_idx in range(LED_COUNT):
        pixels_set(led_idx, color)
    
    # Calculate brightness based on pulse value (linear scaling)
    # Formula: normalize 16-bit sensor value to 0.0-1.0 range
    brightness = pulse_value / BRIGHTNESS_DIVISOR
    
    # Clamp brightness to valid range [0.0, 1.0]
    brightness = max(0.0, min(1.0, brightness))
    
    # Update LED display with pulse-modulated brightness
    pixels_show(brightness)


# ============================================================================
# COLOR DEFINITIONS
# ============================================================================

# Define commonly used RGB colors
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
    Map a value from one range to another using linear interpolation.
    
    Useful for scaling sensor values to different ranges.
    
    Args:
        x (float): Input value to map
        in_min (float): Minimum of input range
        in_max (float): Maximum of input range
        out_min (float): Minimum of output range
        out_max (float): Maximum of output range
        
    Returns:
        float: Mapped value in the output range
        
    Example:
        # Map ADC value (0-65535) to voltage (0-3.3V)
        voltage = map_value(adc_value, 0, 65535, 0, 3.3)
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
    
    Algorithm Overview:
    1. Moving average filtering reduces noise
    2. Derivative calculation detects signal slope changes
    3. Adaptive threshold adjusts to signal characteristics
    4. Peak detection combines derivative sign change + threshold + timing checks
    5. BPM calculation uses time between detected peaks
    """
    
    def __init__(self):
        """Initialize all data structures for signal processing."""
        # ========== Moving Average Filter ==========
        # Reduces noise by averaging recent samples
        self.moving_avg_window = [0] * WINDOW_SIZE
        self.moving_avg_idx = 0
        self.moving_average = 0
        self.prev_moving_average = 0
        
        # ========== Derivative Tracking ==========
        # Used for peak detection via sign change detection
        self.current_diff = 0          # Current derivative (change)
        self.prev_diff = 0             # Previous derivative
        self.diff_product = 0          # Product: detects sign change if < 0
        
        # ========== Adaptive Threshold Computation ==========
        # Threshold adapts to changing signal characteristics
        self.max_min_window = [0] * MAX_MIN_WINDOW_SIZE
        self.max_min_idx = 0
        self.adaptive_threshold = 0    # 70% of range + minimum
        
        # ========== Peak Tracking for BPM Calculation ==========
        # Stores timestamps of recent peaks to calculate BPM
        self.peak_timestamps = [0] * PEAK_LIST_SIZE
        self.peak_idx = 0
        self.beat_count = 0            # Total beats detected
        self.last_peak_time = utime.ticks_ms()
        self.current_bpm = 0           # Current BPM estimate
        
        # ========== OLED Update Counter ==========
        # Tracks when to update the OLED display (once per full window)
        self.update_count = 0
    
    def update(self, raw_sensor_value):
        """
        Process a new PPG sensor reading and update all calculated metrics.
        
        This is the main entry point for processing sensor data. It performs
        all signal processing steps in sequence and returns results.
        
        Args:
            raw_sensor_value (int): 16-bit raw value from PPG sensor (0-65535)
            
        Returns:
            dict: Contains the following keys:
                - 'moving_average' (int): Low-pass filtered signal
                - 'threshold' (int): Current adaptive threshold
                - 'peak_detected' (bool): Whether a peak was detected this update
                - 'bpm' (float): Current BPM estimate
                - 'should_update_oled' (bool): Whether to update display
        """
        # Step 1: Update moving average filter with new sample
        self._update_moving_average(raw_sensor_value)
        
        # Step 2: Calculate derivative for peak detection
        self._update_derivatives()
        
        # Step 3: Compute adaptive threshold based on recent signal range
        self._update_adaptive_threshold(raw_sensor_value)
        
        # Step 4: Detect peaks using combined criteria
        peak_detected = self._detect_peak()
        
        return {
            'moving_average': self.moving_average,
            'threshold': self.adaptive_threshold,
            'peak_detected': peak_detected,
            'bpm': self.current_bpm,
            'should_update_oled': self._should_update_oled()
        }
    
    def _update_moving_average(self, raw_value):
        """
        Update the moving average window with new sensor value.
        
        Implements a simple moving average (MA) filter which reduces
        high-frequency noise while maintaining signal shape.
        
        Args:
            raw_value (int): New raw sensor value to add to window
        """
        self.moving_avg_window[self.moving_avg_idx] = raw_value
        self.moving_avg_idx = (self.moving_avg_idx + 1) % WINDOW_SIZE
        
        # Compute average using integer division for efficiency
        self.moving_average = sum(self.moving_avg_window) // WINDOW_SIZE
        
    
    def _update_derivatives(self):
        """
        Calculate the derivative (rate of change) of the smoothed signal.
        
        The first derivative tells us when the signal is increasing or decreasing.
        By checking when the derivative changes sign (product < 0), we detect peaks
        where the signal goes from increasing to decreasing.
        
        Mathematical insight:
        - Peak occurs where d/dt = 0 (derivative crosses zero)
        - Checking prev_diff * current_diff < 0 detects this crossing
        - More robust than checking <= 0 to avoid multiple detections
        """
        self.current_diff = self.moving_average - self.prev_moving_average
        self.diff_product = self.current_diff * self.prev_diff
        
        self.prev_moving_average = self.moving_average
        self.prev_diff = self.current_diff
    
    def _update_adaptive_threshold(self, raw_value):
        """
        Compute adaptive threshold based on recent signal range.
        
        Rather than using a fixed threshold, we adapt to the signal:
        Threshold = (signal_max - signal_min) * 0.7 + signal_min
        
        This approach:
        - Automatically scales with signal amplitude
        - Prevents false peaks from noise
        - Rejects signals that don't reach minimum signal strength
        - The window is also used for OLED waveform visualization
        
        Args:
            raw_value (int): New raw sensor value for threshold calculation
        """
        self.max_min_window[self.max_min_idx] = raw_value
        self.max_min_idx = (self.max_min_idx + 1) % MAX_MIN_WINDOW_SIZE
        
        signal_min = min(self.max_min_window)
        signal_range = max(self.max_min_window) - signal_min
        
        # Threshold at 70% of range above minimum
        self.adaptive_threshold = (signal_range * PEAK_THRESHOLD_PERCENT) + signal_min
    
    def _detect_peak(self):
        """
        Detect a heartbeat peak based on three criteria:
        
        1. Derivative Sign Change: diff_product < 0
           - Indicates signal changed from increasing to decreasing
           - True peak crossing
        
        2. Signal Above Threshold: moving_average >= adaptive_threshold
           - Peak must be strong enough (not just noise)
           - Adapts to changing signal characteristics
        
        3. Sufficient Time Since Last Peak: time_since_last_peak >= TIMING_THRESHOLD
           - Refractory period prevents multiple detections from same beat
           - Typical heartbeat has minimum duration (~300ms at high HR)
        
        Returns:
            bool: True if a new peak was detected, False otherwise
        """
        current_time = utime.ticks_ms()
        time_since_last_peak = utime.ticks_diff(current_time, self.last_peak_time)
        
        # Check all three peak detection criteria
        is_peak_crossing = (self.diff_product < 0)           # Derivative sign change
        is_above_threshold = (self.moving_average >= self.adaptive_threshold)
        is_sufficient_interval = (time_since_last_peak >= TIMING_THRESHOLD)
        
        if is_peak_crossing and is_above_threshold and is_sufficient_interval:
            # Peak detected - update peak tracking data structures
            self.beat_count += 1
            self.last_peak_time = current_time
            self.peak_timestamps[self.peak_idx] = current_time
            self.peak_idx = (self.peak_idx + 1) % PEAK_LIST_SIZE
            
            # Calculate BPM once we have enough peaks for stable estimate
            if self.beat_count >= BEAT_COUNT_MIN:
                self._calculate_bpm()
            
            return True
        
        return False
    
    def _calculate_bpm(self):
        """
        Calculate beats per minute from the last PEAK_LIST_SIZE peaks.
        
        Formula: BPM = (number_of_peaks * 60 seconds * 1000 ms/s) / time_span_ms
        
        This approach:
        - Uses multiple peaks for accuracy
        - Averages out timing jitter
        - Provides real-time BPM as new peaks arrive
        
        Note:
        - Result is multiplied by 1000 to convert milliseconds to seconds
        - Requires at least BEAT_COUNT_MIN beats before first calculation
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
        
        The display is updated once per complete cycle of the max_min_window.
        Since the window is 128 samples (same as display width), updating
        once per window gives smooth, continuous waveform visualization.
        
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
            list: Raw sensor values from max_min_window buffer
        """
        return self.max_min_window


# ============================================================================
# OLED DISPLAY FUNCTIONS
# ============================================================================

def plot_ppg_waveform(oled, waveform_data, resolution_x, resolution_y):
    """
    Plot the PPG waveform on the OLED display.
    
    Normalizes the waveform data to fit within the display resolution
    and draws it as a series of dots for real-time visualization.
    
    Args:
        oled (SSD1306_I2C): OLED display object
        waveform_data (list): Raw sensor values to plot (0-65535)
        resolution_x (int): Display width in pixels
        resolution_y (int): Display height in pixels
        
    Process:
        1. Clear the entire display
        2. Normalize each sensor value to pixel range
        3. Invert Y-axis (OLED has 0 at top, but signals plot upward)
        4. Draw dots at each position
        5. Send to display
    """
    oled.fill(0)  # Clear the display completely
    
    # Plot each point in the waveform
    for x_pos in range(resolution_x):
        if x_pos < len(waveform_data):
            # Normalize sensor value (0-65535) to display pixel range (0 to resolution_y-1)
            normalized = (waveform_data[x_pos] / ADC_MAX_VALUE) * (resolution_y - 1)
            
            # Invert Y coordinate: OLED pixel 0 is at top, but we want signal to plot upward
            y_pos = resolution_y - int(normalized)
            
            # Draw a single character dot at this position
            oled.text('.', x_pos, y_pos)
    
    oled.show()  # Render all updates to the screen


# ============================================================================
# LED COLOR SELECTION
# ============================================================================

def get_led_color_for_bpm(bpm):
    """
    Select LED color based on current heart rate (BPM).
    
    Color coding provides visual feedback on heart rate status:
    - Blue: Low heart rate (< 60 BPM) - potential bradycardia
    - Green: Normal heart rate (60-100 BPM) - healthy range
    - Red: High heart rate (> 100 BPM) - potential tachycardia
    
    Args:
        bpm (float): Current heart rate in beats per minute
        
    Returns:
        tuple: RGB color tuple for LED display
    """
    if bpm < 60:
        return BLUE      # Slow heart rate (bradycardia warning)
    elif bpm > 100:
        return RED       # Elevated heart rate (tachycardia warning)
    else:
        return GREEN     # Normal heart rate


# ============================================================================
# DATA LOGGING AND EXPORT
# ============================================================================

class DataLogger:
    """
    Logs raw PPG data and peak detections to a text file.
    
    Features:
    - Buffer-based writing to minimize disk I/O
    - Timestamps for all entries
    - Peak detection marking
    - Summary statistics on shutdown
    - Memory efficient buffering
    """
    
    def __init__(self, filename="heartbeat_data.txt", buffer_size=50):
        """
        Initialize the data logger.
        
        Args:
            filename (str): Name of the file to write to
            buffer_size (int): Number of entries to buffer before writing
        """
        self.filename = filename
        self.buffer_size = buffer_size
        self.buffer = []
        self.peak_count = 0
        self.data_count = 0
        self._write_header()
    
    def _write_header(self):
        """Write header information to the log file."""
        try:
            with open(self.filename, 'w') as f:
                #f.write("=" * 70 + "\n")
                #f.write("Heart Rate Monitor - Data Log\n")
                #f.write(f"Timestamp: {utime.localtime()}\n")
                #f.write("=" * 70 + "\n")
                f.write("Format: [timestamp_ms] , raw_ppg , moving_avg , threshold , peak , bpm\n")
                #f.write("=" * 70 + "\n")
            print(f"Log file '{self.filename}' created successfully")
        except Exception as e:
            print(f"Error creating log file: {e}")
    
    def log_data(self, timestamp_ms, raw_ppg, moving_avg, threshold, 
                 peak_detected, bpm):
        """
        Add a data entry to the buffer.
        
        Args:
            timestamp_ms (int): Elapsed time in milliseconds
            raw_ppg (int): Raw PPG sensor value
            moving_avg (int): Moving average value
            threshold (int): Adaptive threshold
            peak_detected (bool): Whether a peak was detected
            bpm (float): Current BPM
        """
        peak_marker = "PEAK" if peak_detected else "----"
        entry = f"{timestamp_ms:>10} , {raw_ppg:>6} , {moving_avg:>6} , {threshold:>6} , {peak_marker} , {bpm:>6.1f}\n"
        
        self.buffer.append(entry)
        self.data_count += 1
        if peak_detected:
            self.peak_count += 1
        
        # Flush buffer to disk when it reaches buffer_size
        if len(self.buffer) >= self.buffer_size:
            self.flush()
    
    def flush(self):
        """Write buffered data to disk."""
        if not self.buffer:
            return
        
        try:
            with open(self.filename, 'a') as f:
                for entry in self.buffer:
                    f.write(entry)
            print(f"Flushed {len(self.buffer)} entries to {self.filename}")
            self.buffer = []
        except Exception as e:
            print(f"Error writing to log file: {e}")
    
    def stop_logging(self):
        """Finalize logging and write summary."""
        self.flush()
        
        try:
            with open(self.filename, 'a') as f:
                f.write("=" * 70 + "\n")
                f.write(f"Total data points logged: {self.data_count}\n")
                f.write(f"Total peaks detected: {self.peak_count}\n")
                f.write(f"Logging stopped at: {utime.localtime()}\n")
                f.write("=" * 70 + "\n")
            print(f"\nLogging stopped. Summary written to {self.filename}")
        except Exception as e:
            print(f"Error writing summary: {e}")


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """
    Main event loop: continuously read sensor data and update LEDs/OLED.
    
    This is the primary runtime loop that:
    1. Reads PPG sensor values
    2. Processes signals for heart rate detection
    3. Updates LED color and brightness
    4. Updates OLED waveform display
    5. Logs all data to file
    
    Press Ctrl+C to gracefully stop the monitor.
    """
    print("\n" + "=" * 60)
    print("Heart Rate Monitor - Initialized")
    print("=" * 60)
    print(f"LED Count: {LED_COUNT}")
    print(f"OLED Resolution: {OLED_RES_X}x{OLED_RES_Y}")
    print(f"Monitoring PPG sensor on ADC pin {PPG_ADC_PIN}")
    print("Waiting for heartbeat detection...\n")
    
    # Initialize signal processor and data logger
    ppg_processor = PPGSignalProcessor()
    #data_logger = DataLogger("heartbeat_data.txt", buffer_size=50)
    
    # Track start time for relative timestamps
    start_time = utime.ticks_ms()
    
    # Main loop - runs indefinitely until user interrupt
    try:
        while True:
            # Get current timestamp relative to start time
            current_time = utime.ticks_ms()
            elapsed_ms = utime.ticks_diff(current_time, start_time)
            
            # Read raw PPG sensor value (16-bit)
            raw_ppg_value = pulse_sensor.read_u16()
            
            # Process the sensor reading through signal processor
            result = ppg_processor.update(raw_ppg_value)
            
            # Log the data for later analysis
            '''
            data_logger.log_data(
                elapsed_ms,
                raw_ppg_value,
                result['moving_average'],
                result['threshold'],
                result['peak_detected'],
                result['bpm']
            )
            '''
            
            # Select LED color based on current heart rate
            led_color = get_led_color_for_bpm(ppg_processor.current_bpm)
            
            # Update LED display with pulse-modulated breathing effect
            breathing_led(led_color, raw_ppg_value)
            
            # Update OLED display once per full waveform cycle
            if result['should_update_oled']:
                plot_ppg_waveform(oled_display, 
                                ppg_processor.get_waveform_data(),
                                OLED_RES_X, OLED_RES_Y)
            
            # Print BPM to console when a new peak is detected
            if result['peak_detected']:
                print(f"♥ Peak detected! BPM: {int(ppg_processor.current_bpm)}")
    
    except KeyboardInterrupt:
        # Graceful shutdown on user interrupt
        print("\n\n" + "=" * 60)
        print("Monitor stopped by user")
        print("=" * 60)
        
        # Finalize data logging
        #data_logger.stop_logging()
        
        # Turn off all LEDs
        set_all_leds(BLACK, 0)
        
        # Update OLED display with shutdown message
        oled_display.fill(0)
        oled_display.text("Monitor stopped", 0, 30)
        oled_display.show()


if __name__ == "__main__":
    main()
