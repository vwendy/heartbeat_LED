This is a prototype project based on two parts that K and W worked on previously, including an LED strip that updates brightness and color periodically, and a heartbeat sensor that reads in real-time heartbeat data. 

We aim to combine the two parts so that the LED strip will reflect the heartbeat in a real-time fashion, which can be further integrated into accessories or clothing as wearable art.

The hardware includes:
1. A microcontroller: raspberry pico that runs MicroPython
2. Pulse sensor for heartbeat data
3. LED strip WS2812
4. OLED screen as an auxilary item to plot data in real-time and troubleshoot sensor errors

The circuit diagram folder has the connectivity diagram of the above hardware components. 

The software is in the "micropython code" folder, including:
1. main.py that reads heartbeat data and show it in the brightness of the LED

There is a "plotting data in R" folder that has an exported txt file as an example and R code to plot the sensor data. This is a longer time frame in general compared to the OLED real-time plotting.

