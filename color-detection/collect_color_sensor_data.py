#!/usr/bin/env python3

"""
This test is used to collect data from the color sensor.
It must be run on the robot.
"""

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor
from time import sleep
from math import sqrt


COLOR_SENSOR_DATA_FILE = "../data_analysis/color_sensor.csv"

# complete this based on your hardware setup
COLOR_SENSOR = EV3ColorSensor(1)
TOUCH_SENSOR = TouchSensor(2)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.


def collect_color_sensor_data():
    "Collect color sensor data."
    try: 
        output_file = open(COLOR_SENSOR_DATA_FILE, "w")
        
        while True:
            if TOUCH_SENSOR.is_pressed():
                color_data = COLOR_SENSOR.get_value()  # Float value in centimeters 0, capped to 255 cm
                if color_data is not None: # If None is given, then data collection failed that time

                    print(color_data)
                    output_file.write("{},{},{}\n".format(color_data[0], color_data[1], color_data[2]))
                sleep(0.5)
    
    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        output_file.close()
        exit()
   
    
    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        exit()
    
    
# For an input RGB value it will return the closest color in the color sensor data by calculating the euclidean distance
# Assumes the input value and the center values are normalized
def calculate_closet_color(r,g,b):
    sample = r,g,b

    # Fill these in with the average RGB values of the color sensor data after collecting it
    blue_center = 0, 0, 255, "Blue"
    red_center = 255, 0, 0, "Red"
    green_center = 0, 255, 0, "Green"
    yellow_center = 255, 255, 0, "Yellow"
    purple_center = 128, 0, 128, "Purple"
    orange_center = 255, 165, 0, "Orange"

    color_center_array = [blue_center, red_center, green_center, yellow_center, purple_center, orange_center]

    # distance, color name
    closest_color = 10000, "NONE"

    for center in color_center_array:
        distance_to_center = calculate_euclidean_distance(sample, center)
        if distance_to_center < closest_color[0]:
            closest_color = distance_to_center, center[3]
    
    print("The cloest color to ", sample, "is ", closest_color[1], "with distance ", closest_color[0])
    return closest_color[1]

def calculate_euclidean_distance(sample, reference):
    sample_r = sample[0]
    sample_g = sample[1]
    sample_b = sample[2]

    reference_r = reference[0]
    reference_g = reference[1]
    reference_b = reference[2]

    distance = sqrt((sample_r - reference_r)**2 + (sample_g - reference_g)**2 + (sample_b - reference_b)**2)

    return  distance

if __name__ == "__main__":
    collect_color_sensor_data()