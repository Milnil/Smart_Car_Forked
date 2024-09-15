import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685
from Line_Tracking import Line_Tracking
from Ultrasonic import Ultrasonic

infrared = Line_Tracking()
ultrasonic = Ultrasonic()
ultrasonic.pwm_S = Servo()
PWM = Motor()
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        while True:
            
            ultrasonic.pwm_S.setServoPwm("0", 90)
            time.sleep(0.1)
            
            obstacle_distance = ultrasonic.get_distance()

            if obstacle_distance < 30: # Check for obstacle close
                ultrasonic.pwm_S.setServoPwm("0", 30)
                time.sleep(0.2)
                Left_dist = ultrasonic.get_distance()
                ultrasonic.pwm_S.setServoPwm("0", 151)
                time.sleep(0.2)
                Right_distance = ultrasonic.get_distance()
                ultrasonic.run_motor(Left_dist, obstacle_distance, Right_distance)
                ultrasonic.pwm_S.setServoPwm("0", 90)
            else: # Run line-tracking
                infrared.LMR=0x00
                if GPIO.input(infrared.IR01)==True:
                    infrared.LMR=(infrared.LMR | 4)
                if GPIO.input(infrared.IR02)==True:
                    infrared.LMR=(infrared.LMR | 2)
                if GPIO.input(infrared.IR03)==True:
                    infrared.LMR=(infrared.LMR | 1)
                if infrared.LMR==2:
                    PWM.setMotorModel(800,800,800,800)
                elif infrared.LMR==4:
                    PWM.setMotorModel(-1500,-1500,2500,2500)
                elif infrared.LMR==6:
                    PWM.setMotorModel(-2000,-2000,4000,4000)
                elif infrared.LMR==1:
                    PWM.setMotorModel(2500,2500,-1500,-1500)
                elif infrared.LMR==3:
                    PWM.setMotorModel(4000,4000,-2000,-2000)
                elif infrared.LMR==7:
                    #pass
                    PWM.setMotorModel(0,0,0,0)
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
        ultrasonic.pwm_S.setServoPwm('0', 90)
