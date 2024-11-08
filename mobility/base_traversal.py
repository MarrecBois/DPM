#V1.0 ROBOT MOTION AROUND THE GRID TO COVER ALL AREAS

import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors

#Allocate resources, initial configuration

BP = brickpi3.BrickPi3()
POWER_LIMIT = 80
SPEED_LIMIT = 200
DPS = 180
DRUM_ANGLE = 20
RIGHT_WHEEL = Motor("B")
LEFT_WHEEL = Motor("C")
US_SENSOR = EV3UltrasonicSensor(3)

# New parameters for spiral
STARTDIST = 15
NUMSPIRALS = 5
INCREMENT = 10
DISTTODEG = 180/(3.1416 * 0.028)
ORIENTTODEG = 0.053/0.02

#Function to initialize motor
def init_motor(motor : Motor):
    try:
        motor.set_limits(POWER_LIMIT, SPEED_LIMIT)
        motor.set_power(0)
    except IOError as error:
        print(error)

#Goes forward until it gets within a certain distance from the wall
def moveUntilHit(distFromWall):
    print(distFromWall)
    current_dist = US_SENSOR.get_value()
    print(current_dist)
    while current_dist == None:
        current_dist = US_SENSOR.get_value()
    
    while current_dist > distFromWall:
        moveDistForward(0.02)
        current_dist = US_SENSOR.get_value()
        print(current_dist)
        while current_dist == None:
            current_dist = US_SENSOR.get_value()
    
    print("OUTSIDE")
    
#Goes forward dist amount of cm
def moveDistForward(dist):
    LEFT_WHEEL.set_position_relative(int(-dist*DISTTODEG))
    RIGHT_WHEEL.set_position_relative(int(-dist*DISTTODEG))

#Turns left by the given angle
def turnLeft(deg):
    try:
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

#Entry point -- print instructions
if __name__=="__main__":
    try:
        wait_ready_sensors(True)
        print('Basic Traversal Test')

        
        try:
            #Initialize the motor and set desired limits
            init_motor(RIGHT_WHEEL)
            init_motor(LEFT_WHEEL)

        except IOError as error:
            print(error)
            
        #Main loop
        dist = STARTDIST
        for i in range(NUMSPIRALS):
            try:
                #ANGLE DEPENDS ON BATTERY
                moveUntilHit(dist)
                print("HI")
                time.sleep(2)
                turnLeft(87)
                time.sleep(2)
                moveUntilHit(dist)
                time.sleep(2)
                turnLeft(87)
                time.sleep(2)
                moveUntilHit(dist)
                time.sleep(2)
                turnLeft(90)
                time.sleep(2)
                dist += INCREMENT
                moveUntilHit(dist)
                time.sleep(2)
                turnLeft(90)
                time.sleep(2)

        
            except IOError as error:
                print(error)

    #Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()
            
