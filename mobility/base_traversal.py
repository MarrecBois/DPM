#V1.0 ROBOT MOTION AROUND THE GRID TO COVER ALL AREAS

import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors

#Allocate resources, initial configuration

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

# New parameters for spiral
STARTDIST = 3
SIDEDIST = 8
NUMSPIRALS = 3
INCREMENT = 15
DISTTODEG = 180/(3.1416 * 0.028)
ORIENTTODEG = 0.053/0.02
DEADBAND = 0.5
DELTASPEED = 100

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

#Entry point -- print instructions
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
