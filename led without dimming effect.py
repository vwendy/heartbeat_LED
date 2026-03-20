###############################################################
# WS2812 RGB LED Ring Light Breathing
# with the Raspberry Pi Pico Microcontroller
#
# by Joshua Hrisko, Maker Portal LLC (c) 2021
#
# Based on the Example neopixel_ring at:
# https://github.com/raspberrypi/pico-micropython-examples
###############################################################
#
import array, time
from machine import Pin
import rp2
from machine import ADC
import utime
import math
#
############################################
# RP2040 PIO and Pin Configurations
############################################
#
# WS2812 LED Ring Configuration
led_count = 11 # number of LEDs in ring light
PIN_NUM = 0 # pin connected to ring light
brightness = 1 # 0.1 = darker, 1.0 = brightest
brightnessv = 0.075 # 0.0 = darker, 1.0 = brightest
speed = 255 # 10, 50, 100 corresponds to BPM being below 80, 80 ~ 100, 100

pulse=ADC(28)

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT,
             autopull=True, pull_thresh=24) # PIO configuration

# define WS2812 parameters
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


# Create the StateMachine with the ws2812 program, outputting on pre-defined pin
# at the 8MHz frequency
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

# Activate the state machine
sm.active(1)

# Range of LEDs stored in an array
ar = array.array("I", [0 for _ in range(led_count)])
#
############################################
# Functions for RGB Coloring
############################################
#
def pixels_show(brightness_input=brightness):
    dimmer_ar = array.array("I", [0 for _ in range(led_count)])
    for ii,cc in enumerate(ar):
        r = int(((cc >> 8) & 0xFF) * brightness_input) # 8-bit red dimmed to brightness
        g = int(((cc >> 16) & 0xFF) * brightness_input) # 8-bit green dimmed to brightness
        b = int((cc & 0xFF) * brightness_input) # 8-bit blue dimmed to brightness
        dimmer_ar[ii] = (g<<16) + (r<<8) + b # 24-bit color dimmed to brightness
    sm.put(dimmer_ar, 8) # update the state machine with new colors
    #time.sleep_ms(10)

def pixels_set(i, color):
    ar[i] = (color[1]<<16) + (color[0]<<8) + color[2] # set 24-bit color
        
#def breathing_led(color):
 #   step = speed
  #  breath_amps = [ii for ii in range(0,255,step)]
   # breath_amps.extend([ii for ii in range(255,-1,-step)])
    #for ii in breath_amps:
     #   for jj in range(len(ar)):
      #      pixels_set(jj, color) # show all colors
       # pixels_show(ii/255)
        #time.sleep(0.02)
        
def breathing_led(color, pulse_value):
    
    step = speed
    breath_amps = [ii for ii in range(0, 255, step)]
    breath_amps.extend([ii for ii in range(255, -1, -step)])
    #for ii in breath_amps:
    
    for jj in range(len(ar)):
        pixels_set(jj, color)  # show all colors
        # Apply intensity factor (clamped between 0 and 1)
    
    
    brightness= math.exp(pulse_value/3000000)
    #print(brightness)
    
    #brightness = (ii / 255) * max(0.0, min(brightnessv, intensity)) #test for pulse input
    #print(brightness)
    pixels_show(brightness)
    #time.sleep(0.02)
    
                
                
        
        

#
############################################
# Main Calls and Loops
############################################
#
# color specifications
red = (255,0,0)
orange = (255,125,0)
yellow = (255,255,0)
spring_green = (125,255,0)
green = (0,255,0)
cyan = (0,255,255)
blue = (0,0,255)
violet = (125,0,255)
magenta = (255,0,255)

pink = (175,0,75)
white = (255,255,255)
blank = (0,0,0)
#colors = [red,orange,yellow,spring_green,green,cyan,blue,violet,magenta]
colors = [red, blue, green]

beat_count = 0
moving_average = 0
prev_moving_average = 0
x_threshold = 0 # value of pulse should be high enough when it's a peak, need computation
timing_threshold = 300 # only the interval > 250ms can have one peak

# save moving average of 20 data points
window_size = 20 # For simple moving average
window = [0] * window_size
idx = 0

# save data for 200 data points to compute max and min, so that we can have a proper threshold for peak detection
# too short of a window will not have the proper range of pulse value
# too long of a window will be wasteful of RAM
max_min_size = 200
max_min_window = [0] * max_min_size
idx_max_min = 0


# save peak information, including timing of peak, value of peak
peak_list_size = 5
peak_list = [utime.ticks_ms()] * peak_list_size
peak_idx = 0

# record difference in moving average
diff = 0
prev_diff = 0
diff_multi_prev_diff = 0

start_time = utime.ticks_ms()
peak_timing = start_time
bpm = 0
    
    
file = open("peak_detection.txt", "w")
    
while True: # loop indefinitely
     
    # Read raw PPG value (16-bit)
    raw = pulse.read_u16()
    
    
                    
    
    # Add to moving average window
    window[idx] = raw
    idx = (idx + 1) % window_size
    
    
    # Compute moving average
    moving_average = sum(window) // window_size
    #print(moving_average)
    
    
    if bpm <60:
        breathing_led(blue,  raw)
    elif bpm > 100:
        breathing_led(red,  raw)
    else:
        breathing_led(green, raw)
    #print(raw)
    
    '''
    '''
    # Compute derivative (change from last filtered value)
    diff = moving_average - prev_moving_average
    prev_moving_average = moving_average
    
    # compute the mulitiplication of first derivative at and before this time step
    diff_multi_prev_diff = diff * prev_diff
    prev_diff = diff
    
    # compute the threshold of peak value, e.g., 80% of the range of the previous 200 data points
    max_min_window[idx_max_min] =raw
    idx_max_min = (idx_max_min+1)%max_min_size
    x_threshold = (max(max_min_window) - min(max_min_window)) * 0.7 + min(max_min_window)
       
    # collect moving average, threshold computed, time interval from last peak into a txt file   
    #file.write("no peak, "+ str(moving_average) +", "+ str(x_threshold) +", "+ str(utime.ticks_diff(utime.ticks_ms(), peak_timing))+", "+"\n")
  
   # Detect peak (derivative at and before the time point is positive and negative respectively, above threshold, sufficient time interval from last peak)
    if ((diff_multi_prev_diff <= 0  ) & (moving_average >= x_threshold) & (utime.ticks_diff(utime.ticks_ms(), peak_timing) >= timing_threshold )):
        
        beat_count += 1
        peak_timing = utime.ticks_ms()
        peak_list[peak_idx] = peak_timing
        peak_idx = (peak_idx + 1) % peak_list_size
          
        #file.write("peak detected, "+str(moving_average) +", "+ str(x_threshold) +", "+ str(utime.ticks_diff(utime.ticks_ms(), peak_timing))+", "+"\n")
      
        # when the beat count is bigger than peak_list_size, aka no zeros in the peak list, we can computer BPM
        if beat_count >=peak_list_size:
            bpm = (peak_list_size * 60) / utime.ticks_diff(max(peak_list) , min(peak_list))  * 1000            
            print(f"BPM: {int(bpm)}")
                
        