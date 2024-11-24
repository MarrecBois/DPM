#Drumming Mechanism Code

import brickpi3
import time
from utils import brick
from utils.brick import BP, Motor, EV3UltrasonicSensor, wait_ready_sensors

#Program parameters

INIT_TIME = 1

#Allocate resources, initial configuration

BP = brickpi3.BrickPi3()
AUX_MOTOR = BP.PORT_B
POWER_LIMIT = 80
SPEED_LIMIT = 720
DPS = 180
ROTATION_ANGLE = 200
US_SENSOR_MOTOR = Motor("A")
US_SENSOR_FRONT = EV3UltrasonicSensor(4)


#Function to initialize motor
def init_motor(motor : Motor):
    try:
        motor.reset_encoder()
        motor.set_limits(POWER_LIMIT, SPEED_LIMIT)
        motor.set_power(0)
    except IOError as error:
        print(error)

#Function that makes the sensor rotate up or down
def sensor_rotate(direction):
    #Calculate the sleep and limits/position of the motor using the global variables
    time.sleep(ROTATION_ANGLE/DPS)
    US_SENSOR_MOTOR.set_limits(dps = DPS)
    if (direction == "up"):
        US_SENSOR_MOTOR.set_position_relative(ROTATION_ANGLE)
    else:
        US_SENSOR_MOTOR.set_position_relative(-ROTATION_ANGLE)
    time.sleep(ROTATION_ANGLE/DPS)



#Entry point -- print instructions
if __name__=="__main__":
    try:
        print('BrickPi Motor Position Demo:')

        
        try:
            #Initialize the motor and set desired limits
            init_motor(US_SENSOR_MOTOR)
            BP.offset_motor_encoder(AUX_MOTOR, BP.get_motor_encoder(AUX_MOTOR))
            BP.set_motor_limits(AUX_MOTOR, POWER_LIMIT, SPEED_LIMIT)
            BP.set_motor_power(AUX_MOTOR, 0)
        except IOError as error:
            print(error)
            
        #Main loop
        
        try:
            sensor_rotate("up")
            sensor_rotate("down")

        
        except IOError as error:
            print(error)

    #Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()

# Current distance is the sampled when the sensor is at the bottom
# Threshold is the distance that is considered a block
def isBlock(current_distance, threshold):
    # Rotate the sensor to the top
    sensor_rotate("up")
    time.sleep(1)

    # Sample from the top position sensor
    top_distance = US_SENSOR_FRONT.get_value()

    # Rotate the sensor to the bottom
    sensor_rotate("down")
    time.sleep(1)

    # Compare the two samples
    if (abs(current_distance - top_distance)) > threshold:
        print("Block detected, current distance is ", current_distance, " and top distance is ", top_distance)
        return True
    else:
        print("No block detected current distance is ", current_distance, " and top distance is ", top_distance)
        return False



