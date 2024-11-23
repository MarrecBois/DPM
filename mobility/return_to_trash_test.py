from base_traversal import *

if __name__=="__main__":
    try:
        wait_ready_sensors(True)
        print('Basic Traversal Test')

        try:
            #Initialize the motor and set desired limits
            init_motor(RIGHT_WHEEL)
            init_motor(LEFT_WHEEL)
            init_motor(US_MOTOR)
     
            goToTrash()

        except IOError as error:
            print(error)      
            
    #Trigger program exit with ^C
    except KeyboardInterrupt:
        BP.reset_all()
        exit()