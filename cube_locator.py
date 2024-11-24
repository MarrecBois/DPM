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
SPEED_LIMIT = 100
DPS = 180
DRUM_ANGLE = 20
RIGHT_WHEEL = Motor("B")
LEFT_WHEEL = Motor("C")
US_MOTOR = Motor("A")
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
DISTTODEG = -180/(3.1416 * 0.028)
DISTFROMCUBE = 3
DISTTOLOCATE = 15
ORIENTTODEG = 0.053/0.02
DEADBAND = 0.5
DELTASPEED = 100
SLEEP_TIME = 0.5
CUBE_DETECT_ANGLE = 5

#Function to initialize motor
def init_motor(motor : Motor):
    try:
        motor.set_limits(POWER_LIMIT, SPEED_LIMIT)
    except IOError as error:
        print(error)

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
        LEFT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
        RIGHT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
        RIGHT_WHEEL.set_position_relative(int(deg*ORIENTTODEG))
        LEFT_WHEEL.set_position_relative(int(-deg*ORIENTTODEG))
    except IOError as error:
        print(error)
        
#Call this function to move forward by x cm
def move_forward(distance):
    LEFT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
    RIGHT_WHEEL.set_limits(POWER_LIMIT, SPEED_LIMIT)
    RIGHT_WHEEL.set_position_relative(int(distance * DISTTODEG))
    LEFT_WHEEL.set_position_relative(int(distance * DISTTODEG))
        
# Call this function to get the bot locked on to a cube's location and the path it needs to take to get there
def get_location():
    RIGHT_WHEEL.set_dps(0)
    LEFT_WHEEL.set_dps(0)
    # Get the current distance from the cube
    current_dist = US_SENSOR_FRONT.get_value()
    smallest_dist = current_dist
    print("GETTING LOCATION")
    while current_dist == None:
        current_dist = US_SENSOR_FRONT.get_value()
    
    #Now rotate left until distance starts increasing, keep track of smallest distance as you go
    num_turns = 1
    print("CHECKING LEFT")
    turnLeft(CUBE_DETECT_ANGLE)
    time.sleep(1)
    current_dist = US_SENSOR_FRONT.get_value()
    print(current_dist)
    while current_dist == None or current_dist == 0:
        current_dist = US_SENSOR_FRONT.get_value()
    
    while current_dist <= smallest_dist + 0.1 and num_turns < 20:
        smallest_dist = current_dist        # This is new smallest distance to the cube
        turnLeft(CUBE_DETECT_ANGLE)                         # Continue turning and check again
        time.sleep(1)
        num_turns += 1
        current_dist = US_SENSOR_FRONT.get_value()
        print(current_dist)
        while current_dist == None or current_dist == 0:
            current_dist = US_SENSOR_FRONT.get_value()
    
    turnRight(CUBE_DETECT_ANGLE) #Go back to where it was the smallest
    time.sleep(1)

    #Now rotate right until distance starts increasing, keep track of smallest distance as you go
    print("CHECKING RIGHT")
    num_turns = 1
    current_dist = US_SENSOR_FRONT.get_value()
    print(current_dist)
    while current_dist == None or current_dist == 0:
        current_dist = US_SENSOR_FRONT.get_value()

    while current_dist <= smallest_dist + 0.1 and num_turns < 20:
        smallest_dist = current_dist        # This is new smallest distance to the cube
        turnRight(CUBE_DETECT_ANGLE)
        time.sleep(1)
        num_turns += 1
        current_dist = US_SENSOR_FRONT.get_value()
        print(current_dist)
        while current_dist == None or current_dist == 0:
            current_dist = US_SENSOR_FRONT.get_value()
    
    turnLeft(CUBE_DETECT_ANGLE) #Go back to where it was the smallest
    time.sleep(1)

    result = "The distance to the bot is: {}".format(smallest_dist)
    return result

def set_speed():
    RIGHT_WHEEL.set_dps(-SPEED_LIMIT)
    LEFT_WHEEL.set_dps(-SPEED_LIMIT)
    print("hey")

# Call this function to get the bot to approach the cube until it is x cm away
def approach_cube():
    turnLeft(20)
    time.sleep(0.5)
    set_speed()
    #Get the current distance from the cube
    current_dist = US_SENSOR_FRONT.get_value()

    #print(current_dist)
    while current_dist == None or current_dist == 0:
        current_dist = US_SENSOR_FRONT.get_value()


    #Repeat until the bot is close to cube
    print("Going for cube now!")
    while current_dist > DISTFROMCUBE:
        current_dist = US_SENSOR_FRONT.get_value()
        turned = False
        #print(current_dist)
        while current_dist == None or current_dist == 0:
            current_dist = US_SENSOR_FRONT.get_value()
        
        #If it's not perfectly lined up, it may need to correct to the right a bit as it approaches the cube...
        while current_dist > DISTTOLOCATE + 3:
            turnRight(2)
            time.sleep(0.1)
            turned = True
            current_dist = US_SENSOR_FRONT.get_value()
            print("approaching distance: {}".format(current_dist))
        
        if turned:
            print("good")
            
        set_speed()


    RIGHT_WHEEL.set_dps(0)
    LEFT_WHEEL.set_dps(0)
    print(current_dist)
    print("I'm here to pick u up u little shit")


#Call this function to turn the appropriate amount to get color sensor directly above the cube to sense it
def turn_and_scan_cube():
    turnLeft(40)
    time.sleep(0.45)
    move_forward(0.5)
    time.sleep(1.7)  


#Entry point -- print instructions
if __name__=="__main__":
    try:
        wait_ready_sensors(True)
        print('Cube locator test')

        
        try:
            #Initialize the motor and set desired limits
            init_motor(RIGHT_WHEEL)
            init_motor(LEFT_WHEEL)
            init_motor(US_MOTOR)

        except IOError as error:
            print(error)
        
        try:
            set_speed()

            picked_up = False

            while not picked_up:
                us_distance = US_SENSOR_FRONT.get_value()
                while us_distance == None:
                    us_distance = US_SENSOR_FRONT.get_value()
                
                print(us_distance)
                if us_distance < DISTTOLOCATE:
                    get_location()
                    approach_cube()
                    print("ready to turn to cube")
                    turn_and_scan_cube()
                    picked_up = True

            RIGHT_WHEEL.set_dps(0)
            LEFT_WHEEL.set_dps(0)

    
        except IOError as error:
            print(error)

    #Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()
        exit()
