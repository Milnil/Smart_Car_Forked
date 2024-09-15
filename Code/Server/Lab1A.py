import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685

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
        # Initialize Motor and Servo
        self.PWM = Motor()
        self.pwm_S = Servo()
        self.M = 0

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

    def run_motor(self, L, M, R):
        if (L < 30 and M < 30 and R < 30) or M < 30:
            self.PWM.setMotorModel(-1000, -1000, -1000, -1000)
            time.sleep(0.1)
            if L < R:
                self.PWM.setMotorModel(1000, 1000, -1000, -1000)
            else:
                self.PWM.setMotorModel(-1000, -1000, 1000, 1000)
        elif L < 30 and M < 30:
            self.PWM.setMotorModel(1500, 1500, -1500, -1500)
        elif R < 30 and M < 30:
            self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
        elif L < 20:
            self.PWM.setMotorModel(1500, 1500, -500, -500)
            if L < 10:
                self.PWM.setMotorModel(1500, 1500, -1000, -1000)
        elif R < 20:
            self.PWM.setMotorModel(-500, -500, 1500, 1500)
            if R < 10:
                self.PWM.setMotorModel(-1000, -1000, 1000, 1000)
        else:
            self.PWM.setMotorModel(600, 600, 600, 600)

    def obstacle_avoidance(self):
        self.pwm_S.setServoPwm("0", 30)
        time.sleep(0.2)
        L = self.get_distance()
        self.pwm_S.setServoPwm("0", 151)
        time.sleep(0.2)
        R = self.get_distance()
        self.run_motor(L, self.M, R)
        self.pwm_S.setServoPwm("0", 90)

    def line_tracking(self):
        LMR = self.read_line_sensors()
        if LMR == 2:
            self.PWM.setMotorModel(800, 800, 800, 800)
        elif LMR == 4:
            self.PWM.setMotorModel(-1500, -1500, 2500, 2500)
        elif LMR == 6:
            self.PWM.setMotorModel(-2000, -2000, 4000, 4000)
        elif LMR == 1:
            self.PWM.setMotorModel(2500, 2500, -1500, -1500)
        elif LMR == 3:
            self.PWM.setMotorModel(4000, 4000, -2000, -2000)
        elif LMR == 7:
            self.PWM.setMotorModel(0, 0, 0, 0)
        else:
            self.PWM.setMotorModel(0, 0, 0, 0)

    def run(self):
        while True:
            self.pwm_S.setServoPwm("0", 90)
            time.sleep(0.1)
            self.M = self.get_distance()
            if self.M < 10:
                self.obstacle_avoidance()
            else:
                self.line_tracking()

# Main program logic follows:
if __name__ == '__main__':
    car = CombinedCar()
    print('Program is starting ... ')
    try:
        car.run()
    except KeyboardInterrupt:
        car.PWM.setMotorModel(0, 0, 0, 0)
        car.pwm_S.setServoPwm('0', 90)
        GPIO.cleanup()
