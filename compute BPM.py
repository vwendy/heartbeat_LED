"""
This is the code to computer beat per minute (BPM). The basic logis is as follows:
1. identify peaks and their timing in the pulse sensor real-time data for a period of time, e.g., 5 beats

Specifically, the peak detection involves
1a. compute moving average to avoid noisy signals (e.g., too high/low)
1b. compute the rate of change of the moving average, or first derivative
1c. computer the time elasped from the last peak to ensure two peaks have reasonable time interval

When there is a peak, it should satisfy the following rules
regarding 1a, the moving average should be above 80% of the range of pulse sensor data
regarding 1b, the rate of change should flip from increasing to decreasing before and after the peak, thus the multiplication of rate of change should be negative
regarding 1c, the time interval should be larger than the fastest possible heat beat, e.g., 150ms

2. infer beat per minute from the data collected in step 1


Note here some choices were empirically tuned on the actual heartbeat, including the window size for moving average, the threshold of peak, and the number of peaks identfied
These choices might not work well potentially with a different heartbeat or BPM, so this is up for testing.

The visualization code was written in R in a different document, which will be uploaded in the github repo as well. That code will show the peaks detected and pulse data.

The pulse value and timing are saved in a txt file, and this file might exceed the storage limit if the code is run for too long, e.g., 1 minute. 
"""


import utime
from machine import ADC


# Configure ADC (e.g., GP26 = ADC0 on Pico)
adc = ADC(28)


beat_count = 0
moving_average = 0
prev_moving_average = 0
x_threshold = 0 # value of pulse should be high enough when it's a peak, need computation
timing_threshold = 150 # only the interval > 250ms can have one peak

# save moving average of 20 data points
window_size = 25 # For simple moving average
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

file = open("peak_detection.txt", "w")

start_time = utime.ticks_ms()
peak_timing = start_time
bpm = 0


counter = 0

while counter <= 4000:
    # limit the data to 4000 data points so the txt file won't excee the storage limit
    counter += 1
    
    # Read raw PPG value (16-bit)
    raw = adc.read_u16()
    
    # Add to moving average window
    window[idx] = raw
    idx = (idx + 1) % window_size
    
    
    # Compute moving average
    moving_average = sum(window) // window_size
    #print(filtered)

    # Compute derivative (change from last filtered value)
    diff = moving_average - prev_moving_average
    prev_moving_average = moving_average
    
    # compute the mulitiplication of first derivative at and before this time step
    diff_multi_prev_diff = diff * prev_diff
    prev_diff = diff
    
    # compute the threshold of peak value, e.g., 80% of the range of the previous 200 data points
    max_min_window[idx_max_min] =raw
    idx_max_min = (idx_max_min+1)%max_min_size
    x_threshold = (max(max_min_window) - min(max_min_window)) * 0.8 + min(max_min_window)
    
    # collect moving average, threshold computed, time interval from last peak into a txt file   
    file.write("no peak, "+ str(moving_average) +", "+ str(x_threshold) +", "+ str(utime.ticks_diff(utime.ticks_ms(), peak_timing))+", "+"\n")
   
   # Detect peak (derivative at and before the time point is positive and negative respectively, above threshold, sufficient time interval from last peak)
    if ((diff_multi_prev_diff < 0  ) & (moving_average >= x_threshold) & (utime.ticks_diff(utime.ticks_ms(), peak_timing) >= timing_threshold )):
        
        # collect moving average, threshold computed, time interval from last peak into a txt file
        file.write("peak detected, "+str(moving_average) +", "+ str(x_threshold) +", "+ str(utime.ticks_diff(utime.ticks_ms(), peak_timing))+", "+"\n")
      
        beat_count += 1
        peak_timing = utime.ticks_ms()
        peak_list[peak_idx] = peak_timing
        peak_idx = (peak_idx + 1) % peak_list_size
          
        # when the beat count is bigger than peak_list_size, aka no zeros in the peak list, we can computer BPM
        if beat_count >=peak_list_size:
            bpm = (peak_list_size * 60) / utime.ticks_diff(max(peak_list) , min(peak_list))  * 1000            
            print(f"BPM: {int(bpm)}")
        
        
