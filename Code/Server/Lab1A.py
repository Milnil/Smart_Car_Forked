import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685
from Line_Tracking import Line_Tracking
from Ultrasonic import Ultrasonic
import random
import logging

class SmartCar:
    def __init__(self):
        self.PWM = Motor()
        self.pwm_S = Servo()
        self.infrared = Line_Tracking()
        self.ultrasonic = Ultrasonic()
        self.min_distance = 30  # Minimum safe distance in cm
        
        # Setup logging
        logging.basicConfig(level=logging.INFO, 
                            format='%(asctime)s - %(levelname)s - %(message)s',
                            handlers=[
                                logging.FileHandler("smart_car.log"),
                                logging.StreamHandler()
                            ])
        self.logger = logging.getLogger("SmartCar")

    def get_line_status(self):
        self.LMR = 0x00
        if GPIO.input(self.infrared.IR01):
            self.LMR |= 4
        if GPIO.input(self.infrared.IR02):
            self.LMR |= 2
        if GPIO.input(self.infrared.IR03):
            self.LMR |= 1
        return self.LMR

    def follow_line(self):
        line_status = self.get_line_status()
        if line_status == 2:
            self.PWM.setMotorModel(800, 800, 800, 800)  # Forward
            self.logger.info("Line detected in center. Moving forward.")
        elif line_status == 4:
            self.PWM.setMotorModel(-1500, -1500, 2500, 2500)  # Turn left
            self.logger.info("Line detected on left. Turning left.")
        elif line_status == 1:
            self.PWM.setMotorModel(2500, 2500, -1500, -1500)  # Turn right
            self.logger.info("Line detected on right. Turning right.")
        elif line_status == 0:
            self.PWM.setMotorModel(800, 800, 800, 800)  # Forward if no line detected
            self.logger.warning("No line detected. Moving forward slowly.")
        elif line_status == 7:
            self.PWM.setMotorModel(0, 0, 0, 0)  # Stop if all sensors triggered
            self.logger.warning("All sensors triggered. Possible intersection or wide line. Stopping.")
        else:
            self.PWM.setMotorModel(0, 0, 0, 0)  # Stop if uncertain
            self.logger.warning(f"Unexpected line status: {line_status}. Stopping.")

    def avoid_obstacle(self):
        self.logger.info("Starting obstacle avoidance maneuver.")
        self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
        self.logger.info("Stopped.")
        time.sleep(0.5)
        self.PWM.setMotorModel(-1000, -1000, -1000, -1000)  # Back up
        self.logger.info("Backing up.")
        time.sleep(0.5)
        # Turn randomly left or right
        turn_direction = random.choice([-1, 1])
        turn_direction_str = "left" if turn_direction == -1 else "right"
        self.PWM.setMotorModel(turn_direction * 2000, turn_direction * 2000, 
                               -turn_direction * 2000, -turn_direction * 2000)
        self.logger.info(f"Turning {turn_direction_str}.")
        time.sleep(0.5)
        self.logger.info("Finished obstacle avoidance maneuver.")

    def run(self):
        self.logger.info("Smart car started running.")
        while True:
            # Check for obstacles
            self.pwm_S.setServoPwm("0", 90)  # Set servo to look forward
            time.sleep(0.1)
            distance = self.ultrasonic.get_distance()

            self.logger.debug(f"Current distance to obstacle: {distance} cm")

            if distance < self.min_distance:
                self.logger.warning(f"Obstacle detected at {distance} cm. Avoiding.")
                self.avoid_obstacle()
            else:
                # No obstacle, follow the line
                self.logger.debug("No obstacle detected. Following line.")
                self.follow_line()

            time.sleep(0.05)  # Small delay to prevent CPU overload

if __name__ == '__main__':
    print('Program is starting...')
    try:
        car = SmartCar()
        car.run()
    except KeyboardInterrupt:
        print("Program stopped by user")
        PWM.setMotorModel(0, 0, 0, 0)
        car.pwm_S.setServoPwm('0', 90)
        logging.info("Program terminated by user.")
    except Exception as e:
        logging.exception("An unexpected error occurred:")
        PWM.setMotorModel(0, 0, 0, 0)
        car.pwm_S.setServoPwm('0', 90)