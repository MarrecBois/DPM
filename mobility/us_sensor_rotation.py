# V1.0 ROBOT MOTION AROUND THE GRID TO COVER ALL AREAS

import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors

# Allocate resources, initial configuration

BP = brickpi3.BrickPi3()
POWER_LIMIT = 80
SPEED_LIMIT = 200
DPS = 180
DRUM_ANGLE = 20
US_SENSOR_MOTOR = Motor("A")
US_SENSOR = EV3UltrasonicSensor(3)

# New parameters for spiral
ORIENTTODEG = 0.053 / 0.02


# Function to initialize motor
def init_motor(motor: Motor):
    try:
        motor.set_limits(POWER_LIMIT, SPEED_LIMIT)
        motor.set_power(0)
    except IOError as error:
        print(error)


# Turns left by the given angle
def turnSensor(deg):
    try:
        US_SENSOR_MOTOR.set_position_relative(int(deg * ORIENTTODEG))
    except IOError as error:
        print(error)



# Entry point -- print instructions
if __name__ == "__main__":
    try:
        wait_ready_sensors(True)
        print('Basic US Rotation Test')

        try:
            # Initialize the motor and set desired limits
            init_motor(US_SENSOR_MOTOR)

        except IOError as error:
            print(error)

        try:
            turnSensor(90)
            time.sleep(2)

        except IOError as error:
            print(error)

    # Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()

