#V1.0 ROBOT MOTION AROUND THE GRID TO COVER ALL AREAS

import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors, EV3ColorSensor
from math import sqrt
from statistics import median

#Allocate resources, initial configuration

# Navigation parameters
BP = brickpi3.BrickPi3()
POWER_LIMIT = 150
SPEED_LIMIT = 360
DPS = 180
DRUM_ANGLE = 20
RIGHT_WHEEL = Motor("B")
LEFT_WHEEL = Motor("C")
US_MOTOR = Motor("A")
US_SENSOR_FRONT = EV3UltrasonicSensor(4)
US_SENSOR_RIGHT = EV3UltrasonicSensor(1)

# Color Detection parameters
CSR = EV3ColorSensor(3)
CSL = EV3ColorSensor(2)

# New parameters for spiral
STARTDIST = 3
SIDEDIST = 8
NUMSPIRALS = 3
INCREMENT = 15
DISTTODEG = 180/(3.1416 * 0.028)
ORIENTTODEG = 0.053/0.02
DEADBAND = 0.5
DELTASPEED = 100


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

    color_center_array = [orange_waste_color_center, yellow_waste_color_center, people_color_center, chair_color_center, ground_color_center, grid_color_center, water_color_center, trash_color_center]

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

# input the color sensor and return a median of 10 samples
def color_sensor_filter(color_sensor):
    r_sample_array = []
    g_sample_array = []
    b_sample_array = []
    try:
        while len(r_sample_array) < 20:
            sample = color_sensor.get_value()
            if sample is not None and sample != [0,0,0,0]:
                r_sample_array.append(sample[0])
                g_sample_array.append(sample[1])
                b_sample_array.append(sample[2])
        
        r_median = median(r_sample_array)
        g_median = median(g_sample_array)
        b_median = median(b_sample_array)
        
        return r_median, g_median, b_median


    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        exit()  


# ****** Robot Navigation ******

#Function to initialize motor
def init_motor(motor : Motor):
    try:
        motor.set_limits(POWER_LIMIT, SPEED_LIMIT)
        motor.set_power(0)
    except IOError as error:
        print(error)

#Goes forward until it gets within a certain distance from the wall
def followWallUntilHit(distFromWall, sideDist):
    print(distFromWall)
    current_dist = US_SENSOR_FRONT.get_value()

    RIGHT_WHEEL.set_dps(-SPEED_LIMIT)
    LEFT_WHEEL.set_dps(-SPEED_LIMIT)
    print(current_dist)
    while current_dist == None:
        current_dist = US_SENSOR_FRONT.get_value()
    
    while current_dist > distFromWall:
        current_dist = US_SENSOR_FRONT.get_value()
        print(current_dist)

        # Get the color sensor data
        CSR_r, CSR_g, CSR_b = color_sensor_filter(CSR)
        CSL_r, CSL_g, CSL_b = color_sensor_filter(CSL)

        # Normalize the color sensor data
        CSR_r, CSR_g, CSR_b = normalize_color_sensor_data(CSR_r, CSR_g, CSR_b)
        CSL_r, CSL_g, CSL_b = normalize_color_sensor_data(CSL_r, CSL_g, CSL_b)

        # Get the closest color
        CSR_color = CSR_calculate_closest_color(CSR_r, CSR_g, CSR_b)
        CSL_color = CSL_calculate_closest_color(CSL_r, CSL_g, CSL_b)
        print("**********CSR: ", CSR_color)
        print("**********CSL: ", CSL_color)

        while current_dist == None:
            current_dist = US_SENSOR_FRONT.get_value()
        
        # Get the distance from the side wall and correct accordingly
        side_dist = US_SENSOR_RIGHT.get_value()
        while side_dist == None:
            side_dist = US_SENSOR_RIGHT.get_value()
        
        error = sideDist - side_dist
        
        if abs(error) < DEADBAND:
            print("ALL GOOD")
            RIGHT_WHEEL.set_dps(-SPEED_LIMIT)
            LEFT_WHEEL.set_dps(-SPEED_LIMIT)

        elif error < 0:
            print("TOO FAR SPEED UP LEFT WHEEL")
            RIGHT_WHEEL.set_dps(-SPEED_LIMIT)
            LEFT_WHEEL.set_dps(-SPEED_LIMIT - DELTASPEED)
        
        else:
            print("TOO CLOSE SPEED UP RIGHT WHEEL")
            RIGHT_WHEEL.set_dps(-SPEED_LIMIT - DELTASPEED)
            LEFT_WHEEL.set_dps(-SPEED_LIMIT)


    RIGHT_WHEEL.set_dps(0)
    LEFT_WHEEL.set_dps(0)
    print("OUTSIDE")
    
#Goes forward dist amount of cm
def moveDistForward(dist):
    LEFT_WHEEL.set_position_relative(int(-dist*DISTTODEG))
    RIGHT_WHEEL.set_position_relative(int(-dist*DISTTODEG))

#Turns left by the given angle
def turnLeft(deg):
    try:
        LEFT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
        RIGHT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
        LEFT_WHEEL.set_position_relative(int(deg*ORIENTTODEG))
        RIGHT_WHEEL.set_position_relative(int(-deg*ORIENTTODEG))
    except IOError as error:
        print(error)

#Turns right by the given angle
def turnRight(deg):
    try:
        RIGHT_WHEEL.set_position_relative(int(deg*ORIENTTODEG))
        LEFT_WHEEL.set_position_relative(int(-deg*ORIENTTODEG))
    except IOError as error:
        print(error)

# ***** Main *****
if __name__=="__main__":
    try:
        wait_ready_sensors(True)
        print('Basic Traversal Test')

        
        try:
            #Initialize the motor and set desired limits
            init_motor(RIGHT_WHEEL)
            init_motor(LEFT_WHEEL)
            init_motor(US_MOTOR)

        except IOError as error:
            print(error)
            
        #Main loop
        dist = STARTDIST
        side = SIDEDIST
        for i in range(NUMSPIRALS):
            try:
                #ANGLE DEPENDS ON BATTERY
                followWallUntilHit(dist, side)
                print("turning left")
                turnLeft(60)
                time.sleep(1)
                moveDistForward(0.3)
                time.sleep(0.5)
                turnLeft(20)
                time.sleep(1)
                followWallUntilHit(dist, side)
                turnLeft(60)
                time.sleep(1)
                moveDistForward(0.3)
                time.sleep(0.5)
                turnLeft(20)
                time.sleep(1)
                followWallUntilHit(dist, side)
                turnLeft(60)
                time.sleep(1)
                moveDistForward(0.3)
                time.sleep(0.5)
                turnLeft(20)
                time.sleep(1)
                dist += INCREMENT
                followWallUntilHit(dist, side)
                turnLeft(60)
                time.sleep(1)
                moveDistForward(0.3)
                time.sleep(0.5)
                turnLeft(20)
                time.sleep(1)
                side += INCREMENT
                

        
            except IOError as error:
                print(error)

    #Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()
        exit()



