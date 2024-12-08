from buildhat_alternative.buildhat import BuildHat
from buildhat_alternative.motor import Motor
from buildhat_alternative.robot import Robot
import time
import signal
import sys
from sshkeyboard import listen_keyboard
import math

class StallDetector():
   def __init__(self,PID_controller):
        self.stalled = False
        self.error_not_decreasing_count = 0
        self.stalled = False
        self.previous_speed = 0
        self.previous_error = 0
        self.PID_controller = PID_controller
      
   def abs_error_is_decreasing(self):
        return abs(self.PID_controller.error)<abs(self.previous_error)
   
   def magnitude_of_speed_above_set_point(self,speed):
        return abs(speed)>abs(self.PID_controller.set_point)
   
   def update(self,speed):
        error = self.PID_controller.error
        if(self.abs_error_is_decreasing() or self.magnitude_of_speed_above_set_point(speed)):
            self.error_not_decreasing_count=0
        else:
            self.error_not_decreasing_count+=1
        if(abs(speed)<abs(self.PID_controller.set_point/2) and self.error_not_decreasing_count>=2):
            self.stalled = True
        else:
            self.stalled = False
        self.previous_error = error
    
         

print('dont press anything yet we will let you know when we are ready to rock.....')

with (
    BuildHat() as buildhat,
    Motor(port="C", ser=buildhat, direction=-1) as left_motor,
):
    try:
        
        #you can add a callback function that will be passed the data that is output from the motor i.e. 
        # the encoder readings,  
        # the motor outputs data pretty quick. note the output_data_rate (its in Hz) parameter above when setting up
        # output data rate must be 100hz or bellow.
        
        # def stop(data):
        #    print(data)
        #    if(left_motor.isStalled()):
        #        print('stalled')
        #        left_motor.run(0)
        
        stall_detector = StallDetector(left_motor.PIDcontroller)
        
        def cb(motor):
            stall_detector.update(left_motor.data["speed_deg/sec"])
            if stall_detector.stalled:
                print('stalled')
                left_motor.run(0)
            pass
        
        def just_a(motor):
            print(motor.data)
        
        left_motor.run(degrees_per_second=250)
       
        left_motor.add_listener(just_a,100,5)
        time.sleep(1)
        left_motor.add_listener(cb,10,0)
        time.sleep(10)
            
        left_motor.run(0)
       
    except KeyboardInterrupt:
        print("you pressed control c")

