from buildhat_alternative.buildhat import BuildHat
from buildhat_alternative.motor import Motor
from buildhat_alternative.robot import Robot
import time
import signal
import sys
from sshkeyboard import listen_keyboard
import math

print('dont press anything yet we will let you know when we are ready to rock.....')

with (
    BuildHat() as buildhat,
    Motor(port="C", ser=buildhat, direction=-1,output_data_rate=1) as left_motor,
):
    try:
        
        #you can add a callback function that will be passed the data that is output from the motor i.e. 
        # the encoder readings,  
        # the motor outputs data pretty quick. note the output_data_rate (its in Hz) parameter above when setting up
        # output data rate must be 100hz or bellow.
        
        def stop(data):
           print(data)
           if(left_motor.isStalled()):
               print('stalled')
               left_motor.run(0)
        
        
        
        left_motor.run(degrees_per_second=180)
       
        left_motor.add_listener(stop)
        time.sleep(10)
            
        left_motor.run(0)
       
    except KeyboardInterrupt:
        print("you pressed control c")

