import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685

class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)
    
    def read_sensors(self):
        LMR = 0x00
        if GPIO.input(self.IR01):
            LMR |= 4
        if GPIO.input(self.IR02):
            LMR |= 2
        if GPIO.input(self.IR03):
            LMR |= 1
        return LMR
    
    def control_motors(self, LMR):
        if LMR == 2:
            PWM.setMotorModel(800, 800, 800, 800)  # Go forward
        elif LMR == 4:
            PWM.setMotorModel(-1500, -1500, 2500, 2500)  # Turn left
        elif LMR == 6:
            PWM.setMotorModel(-2000, -2000, 4000, 4000)  # Turn left sharply
        elif LMR == 1:
            PWM.setMotorModel(2500, 2500, -1500, -1500)  # Turn right
        elif LMR == 3:
            PWM.setMotorModel(4000, 4000, -2000, -2000)  # Turn right sharply
        elif LMR == 7:
            PWM.setMotorModel(0, 0, 0, 0)  # Stop

class Ultrasonic:
    def __init__(self):
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # cm
        self.timeOut = self.MAX_DISTANCE * 60  # Timeout calculation
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.pwm_S = Servo()

    def get_distance(self):
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, GPIO.LOW)
        
        start_time = time.time()
        while GPIO.input(self.echo_pin) == GPIO.LOW:
            if time.time() - start_time > self.timeOut:
                return self.MAX_DISTANCE

        start = time.time()
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            if time.time() - start > self.timeOut:
                return self.MAX_DISTANCE

        distance = (time.time() - start) * 17150
        return round(distance, 2)
    
    def avoid_obstacle(self, reverse=False):
        distance = self.get_distance()
        if distance < 30:
            # Obstacle detected: reverse if requested
            if reverse:
                PWM.setMotorModel(-1500, -1500, -1500, -1500)  # Back up
                time.sleep(1)

            # Turn and attempt to avoid obstacle
            self.pwm_S.setServoPwm('0', 30)  # Turn left
            time.sleep(0.2)
            left_distance = self.get_distance()
            
            self.pwm_S.setServoPwm('0', 151)  # Turn right
            time.sleep(0.2)
            right_distance = self.get_distance()

            # Choose the direction with more space
            if left_distance > right_distance:
                PWM.setMotorModel(-1500, -1500, 2500, 2500)  # Turn left
            else:
                PWM.setMotorModel(2500, 2500, -1500, -1500)  # Turn right
            
            self.pwm_S.setServoPwm('0', 90)  # Reset to center
        else:
            return False  # No obstacle
        return True

# Main program that integrates line tracking and obstacle avoidance
if __name__ == '__main__':
    print('Program is starting ...')
    line_tracking = Line_Tracking()
    ultrasonic = Ultrasonic()

    try:
        while True:
            # Check for obstacles first
            obstacle_detected = ultrasonic.avoid_obstacle()
            
            # Check line tracking status
            LMR = line_tracking.read_sensors()

            if not obstacle_detected:
                if LMR != 0:  # If there is a line to follow
                    line_tracking.control_motors(LMR)
                else:
                    print("Line lost, reversing and avoiding obstacle...")
                    PWM.setMotorModel(-1500, -1500, -1500, -1500)  # Back up
                    time.sleep(1)
                    # Then check for obstacles while backing up
                    ultrasonic.avoid_obstacle(reverse=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        PWM.setMotorModel(0, 0, 0, 0)
        ultrasonic.pwm_S.setServoPwm('0', 90)
