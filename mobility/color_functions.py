import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors, EV3ColorSensor
from math import sqrt
from statistics import median
from cube_locator import *

#Allocate resources, initial configuration

# US Sensor Rotation Parameters
US_MOTOR = Motor("A")
DPS = 180
ROTATION_ANGLE = 200

# Navigation parameters
BP = brickpi3.BrickPi3()
POWER_LIMIT = 150
SPEED_LIMIT = 360
DRUM_ANGLE = 20
RIGHT_WHEEL = Motor("B")
LEFT_WHEEL = Motor("C")
US_SENSOR_FRONT = EV3UltrasonicSensor(4)
US_SENSOR_RIGHT = EV3UltrasonicSensor(1)

# Color Detection parameters
CSR = EV3ColorSensor(2)
CSL = EV3ColorSensor(3)
MIN_WATER_DIST_TO_WALL = 5

# New parameters for spiral
STARTDIST = 4
SIDEDIST = 9
NUMSPIRALS = 3
INCREMENT = 15
DISTTODEG = 180/(3.1416 * 0.028)
ORIENTTODEG = 0.053/0.02
DEADBAND = 0.5
DELTASPEED = 100
SLEEP_TIME = 0.5
# ****** Color Detection ******

# For an input RGB value it will return the closest color in the color sensor data by calculating the euclidean distance
# Assumes the input value and the center values are NORMALIZED
# For Color Sensor Left (Low)
def CSL_calculate_closest_color(r,g,b):
    sample = r,g,b

    # Ground Colors
    ground_color_center = (0.338, 0.549, 0.113, "GROUND") # Green / unormalized (92.52, 150.33, 30.90)
    grid_color_center = (0.796, 0.112, 0.091 , "GRID") # Red / unormalized (223.46, 31.54, 25.67)
    water_color_center = (0.202, 0.255, 0.543, "WATER") # Blue / unormalized (31.00, 39.21, 83.29)
    trash_color_center = (0.545, 0.379, 0.076, "TRASH") # Yellow / unormalized (322.42, 224.46, 45.00)

    color_center_array = [grid_color_center, water_color_center, trash_color_center, ground_color_center]

    # distance, color name
    closest_color = 10000, "NONE"

    for center in color_center_array:
        distance_to_center = calculate_euclidean_distance(sample, center)
        if distance_to_center < closest_color[0]:
            closest_color = distance_to_center, center[3]
    
    return closest_color[1]

# For an input RGB value it will return the closest color in the color sensor data by calculating the euclidean distance
# Assumes the input value and the center values are NORMALIZED
# For Color Sensor Right (High)
def CSR_calculate_closest_color(r,g,b):
    sample = r,g,b

    # Block colors
    orange_waste_color_center = (0.722, 0.173, 0.106, "ORANGEPOOP") # Orange / unormalized (351.87, 84.22, 51.43)
    yellow_waste_color_center = (0.576, 0.376, 0.048, "YELLOWPOOP") # Yellow / unnormalized (354.92, 231.72, 29.76)
    people_color_center = (0.384, 0.268, 0.348, "PEOPLE") # Purple / unnormalized (83.82, 58.58, 75.94)
    chair_color_center = (0.147, 0.642, 0.211, "CHAIR") # Green / unnormalized (38.72, 169.08, 55.68)

    # Ground Colors
    ground_color_center = (0.342, 0.539, 0.119, "GROUND") # Green / unnormalized (14.00, 22.04, 4.88)
    grid_color_center = (0.693, 0.201, 0.106, "GRID") # Red / unnormalized (28.45, 8.25, 4.35)
    water_color_center = (0.179, 0.251, 0.570, "WATER") # Blue / unnormalized (4.48, 6.30, 14.26)
    trash_color_center = (0.5379, 0.3761, 0.0861, "TRASH") # Yellow / unnormalized (48.00, 33.56, 7.68)
    # Need to retry the trash color center

    color_center_array = [ground_color_center, grid_color_center, water_color_center, trash_color_center, orange_waste_color_center, yellow_waste_color_center, people_color_center, chair_color_center]

    # distance, color name
    closest_color = 10000, "NONE"

    for center in color_center_array:
        distance_to_center = calculate_euclidean_distance(sample, center)
        if distance_to_center < closest_color[0]:
            closest_color = distance_to_center, center[3]
    
    return closest_color[1]

# For an input RGB value it will return the closest color in the color sensor data by calculating the euclidean distance
# Assumes the input value and the center values are NORMALIZED
# For Color Sensor Right (High)
def CSR_calculate_closest_cube(r,g,b):
    sample = r,g,b

    # Block colors
    orange_waste_color_center = (0.722, 0.173, 0.106, "ORANGEPOOP") # Orange / unormalized (351.87, 84.22, 51.43)
    yellow_waste_color_center = (0.576, 0.376, 0.048, "YELLOWPOOP") # Yellow / unnormalized (354.92, 231.72, 29.76)
    people_color_center = (0.384, 0.268, 0.348, "PEOPLE") # Purple / unnormalized (83.82, 58.58, 75.94)
    chair_color_center = (0.147, 0.642, 0.211, "CHAIR") # Green / unnormalized (38.72, 169.08, 55.68)

    color_center_array = [orange_waste_color_center, yellow_waste_color_center, people_color_center, chair_color_center]

    # distance, color name
    closest_color = 10000, "NONE"

    for center in color_center_array:
        distance_to_center = calculate_euclidean_distance(sample, center)
        if distance_to_center < closest_color[0]:
            closest_color = distance_to_center, center[3]
    
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

def normalize_color_sensor_data(r,g,b):
    normalized_r = r/(r+g+b)
    normalized_g = g/(r+g+b)
    normalized_b = b/(r+g+b)

    return normalized_r, normalized_g, normalized_b

# input the color sensor and return a median of 20 samples
def color_sensor_filter(color_sensor):
    r_sample_array = []
    g_sample_array = []
    b_sample_array = []
    try:
        while len(r_sample_array) < 20:
            sample = color_sensor.get_value()
            if color_sensor == CSR:
                print("*****CSR: sample value,", sample)
            if sample is not None and all(value != 0 for value in sample):
                r_sample_array.append(sample[0])
                g_sample_array.append(sample[1])
                b_sample_array.append(sample[2])
            else:
                print("Invalid sample detected, retrying...")
                time.sleep(0.05)  # Small delay before retrying

        r_median = median(r_sample_array)
        g_median = median(g_sample_array)
        b_median = median(b_sample_array)

        return r_median, g_median, b_median

    except IOError as error:
        print(f"I/O error during color sensor filtering: {error}")
        return None, None, None  # Return default values or handle as needed
    except Exception as e:
        print(f"Unexpected error during color sensor filtering: {e}")
        return None, None, None

if __name__ == "__main__":
    print("yo")