## Documents/Thonny/TemperatureControl//PID_TempControl.py

#Author - Anthony Peters -- anthony.peters@sjsu.edu

from machine import Pin, PWM , ADC
import utime, machine, rp2
import time

# INITIALIZE PICO FAN , TEMP , AND CONTROLLER

### FAN

fan = PWM(Pin(20))
tachometer = Pin(26, Pin.OUT)
fan.freq(125) # 100 Hz

def set_fan_speed(percent):
    if percent < 0 or percent > 100:
        raise ValueError("Invalid percentage value. Must be between 0 and 100.") # Ensure the input is a valid percentage
    duty_cycle = int((-220.22*percent) + 91520.22) # Calculate the duty cycle from the percentage
    fan.duty_u16((duty_cycle)) # Set the duty cycle of the PWM signal

### TEMPERATURE
    
ADC_TEMP_CHANNEL = const(4)
temp_sensor = ADC(ADC_TEMP_CHANNEL)
VOLTS_PER_COUNT = 3.3 / ((2**16)-1)
DELAY_SECS = 0.01
timecount = 0

def get_temp(adc):
    v = adc.read_u16() * VOLTS_PER_COUNT
    temp = 27 - ((v-0.706)/0.001721)   # from 4.9.5 in RP2040 datasheet
    return temp

sum = get_temp(temp_sensor)


### CONTORLLER
#Build a Class for the PID Controller allows for efficent tunning

class PIDController:
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0   ##

    def update(self, setpoint, current_temperature):
        error = setpoint - current_temperature
        self.integral += error
        derivative = error - self.previous_error 
        self.previous_error = error
        
        if error > 0:
         PID = idle
         self.integral = 0 # resets integral when setpoint reached
         self.previous_error = 0
         
        elif error > 100:
         PID = fan_fullspeed
         
        else:
         PID = self.kp * error + self.ki * self.integral + self.kd * derivative # PID CONTROLLER
         
        return PID


# Perfomance Criteria

setpoint = 23 #Degree Celcius
fan_duty_cycle = 0

#Initilize PID Controller
pid = PIDController(6, 0.4, 8)
idle = 1  
fan_fullspeed = 100


while True :
    
######### TEMPERATURE  ##########
    ## Averae temp Reading
    if timecount == 20:
        timecount = 0
        sum = get_temp(temp_sensor) # Time has reached 10 seconds now start over
    
    # Temperature readings every 0.5 seconds
    current_temperature = get_temp(temp_sensor)
    avg_moveTemp = (sum / (timecount + 1))
    timecount += 1
    time.sleep(DELAY_SECS) ##take into account nyquist rate.. fs >= Wn
    sum += get_temp(temp_sensor)
    #print ( "[TEMP] Real vs Avg:" , current_temperature, avg_moveTemp )
   
######### FAN ##########

    fan_duty_cycle = abs(pid.update(setpoint , current_temperature))
    if fan_duty_cycle > fan_fullspeed:
        fan_duty_cycle = fan_fullspeed   # Fan speed proportional to duty cycle
        
    set_fan_speed((fan_duty_cycle))
    print ( "Duty Cycle:" , fan_duty_cycle , current_temperature )
    time.sleep(0.5) # Wn












#   ### Fan Control (Loop) ###
#     for i in range(1, 12, 5):
#         # Set the duty cycle of the PWM signal
#         set_fan_speed(i)
#         time.sleep(0.001)
#     for i in range(10, 2, -5):
#         set_fan_speed(i)
#         time.sleep(0.01)
#     
#     ####set_fan_speed(50)









    
