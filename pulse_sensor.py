#              .';:cc;.
#            .,',;lol::c.
#            ;';lddddlclo
#            lcloxxoddodxdool:,.
#            cxdddxdodxdkOkkkkkkkd:.
#          .ldxkkOOOOkkOO000Okkxkkkkx:.
#        .lddxkkOkOOO0OOO0000Okxxxxkkkk:
#       'ooddkkkxxkO0000KK00Okxdoodxkkkko
#      .ooodxkkxxxOO000kkkO0KOxolooxkkxxkl
#      lolodxkkxxkOx,.      .lkdolodkkxxxO.
#      doloodxkkkOk           ....   .,cxO;
#      ddoodddxkkkk:         ,oxxxkOdc'..o'
#      :kdddxxxxd,  ,lolccldxxxkkOOOkkkko,
#       lOkxkkk;  :xkkkkkkkkOOO000OOkkOOk.
#        ;00Ok' 'O000OO0000000000OOOO0Od.
#         .l0l.;OOO000000OOOOOO000000x,
#            .'OKKKK00000000000000kc.
#               .:ox0KKKKKKK0kdc,.
#                      ...
#
# Author: peppe8o
# Date: Oct 15th, 2022
# Version: 1.0
# blog: https://peppe8o.com

# Please rename to "main.py" this script if you want to run it witout a connected PC

from machine import ADC
import time

# setup the Pulse Sensor reading pin
pulse=ADC(28)
file = open("pulse_sensor_data.txt", "w")
x=0


start_time = time.ticks_ms()

# main program
while x<5000:
    x=x+1
    try:
        value=pulse.read_u16()
        timestamp = time.ticks_ms()
        file.write(str(timestamp)+", "+str(value)+"\n")
        
    except OSError as e:
        machine.reset()




end_time = time.ticks_ms()

# Calculate the difference in milliseconds
elapsed_time_ms = time.ticks_diff(end_time, start_time)

print(f"Elapsed time: {elapsed_time_ms} milliseconds")