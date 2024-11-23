#!/usr/bin/env python3

"""
This is a test to see if the color classification works. 
It must be run on the robot.
"""

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor
from time import sleep
from math import sqrt
from collect_color_sensor_data import CSL_calculate_closest_color, CSR_calculate_closest_color, normalize_color_sensor_data

# Sensor initialization
COLOR_SENSOR = EV3ColorSensor(2)
TOUCH_SENSOR = TouchSensor(3)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.

# It will sense the color and return the closest color in the color sensor data by calculating the euclidean distance and normalizing the input depending on the sensor
def test_color_classification(sensor):
    "Collect color sensor data."
    try:         
        while True:
            if TOUCH_SENSOR.is_pressed():
                color_data = COLOR_SENSOR.get_value()  
                if color_data is not None: # If None is given, then data collection failed that time
                    print(color_data)
                    if sensor == "CSL":
                        r,g,b = normalize_color_sensor_data(color_data[0], color_data[1], color_data[2])
                        print(CSL_calculate_closest_color(r,g,b))
                    elif sensor == "CSR":
                        r,g,b = normalize_color_sensor_data(color_data[0], color_data[1], color_data[2])
                        print(CSR_calculate_closest_color(r,g,b))
                    else:
                        print("Sensor not recognized")
                sleep(0.5)
    
    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        exit()

# Test the color classification for the left color sensor
if __name__ == "__main__":
    test_color_classification("CSR")
    

