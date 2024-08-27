class PIDController:
   def __init__(self, 
                 proportional_constant=0, 
                 integral_constant=0,
                 derivative_constant=0,
                 #process_variable=0,#going to remove this
                 power=0):
       
      self.proportional_constant = proportional_constant
      self.integral_constant = integral_constant
      self.derivative_constant = derivative_constant
      # Running sums
      self.integral_sum = 0
      self.previous = 0
      # self.process_variable = process_variable #going to remove this
      self.power = power
      self.set_point = 0
      
      ##stall detection
      self.error_not_decreasing_count = 0
      self.stalled = False
      self.previous_speed = 0
        
   def handle_proportional(self,error):
      return self.proportional_constant * error
   
   ##this is a utility used to adjust the pid values during a refactor
   ## it should get removed once we check we have the math correct
   ##basically we have converted the process_variable from mm_sec to deg per sec
   # so we need to do the inverse to the pid values to make the math work.
   #be carefull there is some ambiguity in whether the wheel diameter is 276.401 or 276
   def deg_per_second_to_mm_per_second(self,deg_per_sec):
      return (1/360)*276*deg_per_sec

  
   def handle_integral(self,error):
      self.integral_sum += error
      return self.integral_constant * error
  
   def handle_derivative(self,error):
      derivative = self.derivative_constant*(error - self.previous)
      self.previous = error
      return derivative

   def get_value(self,error):
      p=self.handle_proportional(error)
      i=self.handle_integral(error)
      d= self.handle_derivative(error)
      return p+i+d
   
   def abs_error_is_decreasing(self,current_error):
      return abs(current_error)<abs(self.previous)
   
   def magnitude_of_speed_above_set_point(self,speed):
      return abs(speed)>abs(self.set_point)
   
   def update(self,speed):
      if(self.set_point==0):
         #consider putting in a reset method to do this
         self.power = 0
         self.integral_sum = 0
         self.previous = 0
         return self.power
      error = speed-self.set_point
      if(self.abs_error_is_decreasing(error) or self.magnitude_of_speed_above_set_point(speed)):
         self.error_not_decreasing_count=0
      else:
         self.error_not_decreasing_count+=1
      if(abs(speed)<abs(self.set_point/2) and self.error_not_decreasing_count>5):
         self.stalled = True
      else:
         self.stalled = False
      adjustment = self.get_value(error)
      self.power = self.power-adjustment
      self.previous_speed = speed
      return max(0.6,self.power)

   
   def set_point(self,set_point):
      self.set_point = set_point
      
      
   
   

