import time
from Motor import *
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
from Led import Led  # Import the Led class

class CombinedCar:
    def __init__(self):
        # Initialize GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # Initialize ultrasonic sensor pins
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300
        self.timeOut = self.MAX_DISTANCE * 60
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        # Initialize line tracking sensor pins
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)
        # Initialize Motor
        self.PWM = Motor()
        self.M = 0
        # Initialize LED
        self.led = Led()

    def pulseIn(self, pin, level, timeOut):
        t0 = time.time()
        while (GPIO.input(pin) != level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0
        t0 = time.time()
        while (GPIO.input(pin) == level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime

    def get_distance(self):
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0
        distance_cm = sorted(distance_cm)
        return int(distance_cm[2])

    def read_line_sensors(self):
        LMR = 0x00
        if GPIO.input(self.IR01):
            LMR |= 4
        if GPIO.input(self.IR02):
            LMR |= 2
        if GPIO.input(self.IR03):
            LMR |= 1
        return LMR

    def obstacle_avoidance(self):
        # Activate LEDs when obstacle is detected
        print("Obstacle detected! Activating LEDs.")
        self.led.colorWipe(self.led.strip, Color(255, 0, 0))  # Red color

        # Measure front distance (already done)
        front_distance = self.M
        print(f"Front distance: {front_distance} cm")

        # Turn car to the left and measure distance
        print("Turning left to scan for obstacle.")
        self.PWM.setMotorModel(-1500, -1500, 1500, 1500)  # Turn left
        time.sleep(0.5)  # Adjust time to turn appropriate angle
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        left_distance = self.get_distance()
        print(f"Left distance: {left_distance} cm")

        # Turn car to the right to scan right
        print("Turning right to scan for obstacle.")
        self.PWM.setMotorModel(1500, 1500, -1500, -1500)  # Turn right
        time.sleep(1.0)  # Adjust time to turn from left position to right position
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        right_distance = self.get_distance()
        print(f"Right distance: {right_distance} cm")

        # Return to center position
        print("Returning to center position.")
        self.PWM.setMotorModel(-1500, -1500, 1500, 1500)  # Turn left
        time.sleep(0.5)  # Adjust time to return to center
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)

        # Decide direction to avoid obstacle
        if left_distance > right_distance:
            # Turn left to avoid obstacle
            print("Choosing to turn left to avoid obstacle.")
            self.PWM.setMotorModel(-1500, -1500, 1500, 1500)  # Turn left
        else:
            # Turn right to avoid obstacle
            print("Choosing to turn right to avoid obstacle.")
            self.PWM.setMotorModel(1500, 1500, -1500, -1500)  # Turn right

        time.sleep(0.5)  # Adjust this delay as needed

        # Move forward to bypass obstacle
        print("Moving forward to bypass obstacle.")
        self.PWM.setMotorModel(1000, 1000, 1000, 1000)
        time.sleep(1)

        # Check if obstacle is still in front
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        self.M = self.get_distance()
        if self.M < 30:
            # Obstacle still present, recursively avoid
            self.obstacle_avoidance()
        else:
            print("Obstacle avoided, deactivating LEDs, resuming line tracking.")
            # Deactivate LEDs after obstacle is avoided
            self.led.colorWipe(self.led.strip, Color(0, 0, 0), 10)  # Turn off LEDs

    def line_tracking(self):
        LMR = self.read_line_sensors()
        if LMR == 2:  # Middle sensor detects line
            self.PWM.setMotorModel(800, 800, 800, 800)
        elif LMR == 4:  # Left sensor detects line
            self.PWM.setMotorModel(-1500, -1500, 2000, 2000)
        elif LMR == 6:  # Left and middle sensors detect line
            self.PWM.setMotorModel(-2000, -2000, 4000, 4000)
        elif LMR == 1:  # Right sensor detects line
            self.PWM.setMotorModel(2000, 2000, -1500, -1500)
        elif LMR == 3:  # Right and middle sensors detect line
            self.PWM.setMotorModel(4000, 4000, -2000, -2000)
        elif LMR == 7:  # All sensors detect line
            self.PWM.setMotorModel(0, 0, 0, 0)
        elif LMR == 0:  # No line detected
            self.PWM.setMotorModel(800, 800, 800, 800)  # Proceed straight
        else:
            self.PWM.setMotorModel(0, 0, 0, 0)

    def run(self):
        while True:
            # Check for obstacles
            time.sleep(0.1)
            self.M = self.get_distance()
            if self.M < 30:
                # Obstacle detected, perform obstacle avoidance
                self.PWM.setMotorModel(0, 0, 0, 0)  # Stop before avoiding
                self.obstacle_avoidance()
                # After avoiding, continue to next loop iteration
                continue
            else:
                # No obstacle detected, perform line tracking
                self.line_tracking()

    def cleanup(self):
        self.PWM.setMotorModel(0, 0, 0, 0)
        # Turn off LEDs during cleanup
        self.led.colorWipe(self.led.strip, Color(0, 0, 0), 10)
        GPIO.cleanup()

# Main program logic follows:
if __name__ == '__main__':
    car = CombinedCar()
    print('Program is starting ... ')
    try:
        car.run()
    except KeyboardInterrupt:
        car.cleanup()
