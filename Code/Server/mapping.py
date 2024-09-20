import time
import RPi.GPIO as GPIO
from Motor import Motor
from Led import Led
from rpi_ws281x import Color
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians


class AdvancedMapping:
    def __init__(
        self, map_size=(200, 200), initial_position=(100, 100), initial_angle=0
    ):
        # Initialize GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Initialize components
        self.PWM = Motor()
        self.led = Led()

        # Ultrasonic sensor pins
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # cm
        self.timeOut = self.MAX_DISTANCE * 60
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

        # Line tracking sensor pins
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

        # Mapping variables
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)

    def pulseIn(self, pin, level, timeOut):
        t0 = time.time()
        while GPIO.input(pin) != level:
            if (time.time() - t0) > timeOut * 0.000001:
                return 0
        t0 = time.time()
        while GPIO.input(pin) == level:
            if (time.time() - t0) > timeOut * 0.000001:
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
        return (
            (GPIO.input(self.IR01) << 2)
            | (GPIO.input(self.IR02) << 1)
            | GPIO.input(self.IR03)
        )

    def obstacle_avoidance(self):
        print("Obstacle detected! Activating LEDs.")
        self.led.colorWipe(self.led.strip, Color(255, 0, 0))  # Red color

        front_distance = self.get_distance()
        print(f"Front distance: {front_distance} cm")

        # Turn left and measure
        self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
        time.sleep(0.5)
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        left_distance = self.get_distance()
        print(f"Left distance: {left_distance} cm")

        # Turn right and measure
        self.PWM.setMotorModel(1500, 1500, -1500, -1500)
        time.sleep(1.0)
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        right_distance = self.get_distance()
        print(f"Right distance: {right_distance} cm")

        # Return to center
        self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
        time.sleep(0.5)
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)

        # Choose direction
        if left_distance > right_distance:
            print("Turning left to avoid obstacle.")
            self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
        else:
            print("Turning right to avoid obstacle.")
            self.PWM.setMotorModel(1500, 1500, -1500, -1500)

        time.sleep(0.5)

        # Move forward
        print("Moving forward to bypass obstacle.")
        self.PWM.setMotorModel(1000, 1000, 1000, 1000)
        time.sleep(1)

        # Check if obstacle is still present
        self.PWM.setMotorModel(0, 0, 0, 0)
        time.sleep(0.1)
        if self.get_distance() < 30:
            self.obstacle_avoidance()
        else:
            print("Obstacle avoided, deactivating LEDs, resuming mapping.")
            self.led.colorWipe(self.led.strip, Color(0, 0, 0), 10)

    def update_map(self, distance, angle):
        rad_angle = radians(self.angle + angle)
        x = int(self.position[0] + distance * cos(rad_angle))
        y = int(self.position[1] + distance * sin(rad_angle))

        if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
            self.map[y, x] = 1  # Mark as obstacle

            for i in range(1, int(distance)):
                ix = int(self.position[0] + i * cos(rad_angle))
                iy = int(self.position[1] + i * sin(rad_angle))
                if 0 <= ix < self.map_size[0] and 0 <= iy < self.map_size[1]:
                    self.map[iy, ix] = 0.5  # Mark as free space

    def scan_environment(self):
        for angle in range(-90, 91, 15):  # Scan from -90 to 90 degrees
            self.PWM.setMotorModel(-1500, -1500, 1500, 1500)  # Rotate left
            time.sleep(0.1)
            self.PWM.setMotorModel(0, 0, 0, 0)
            distance = self.get_distance()
            self.update_map(distance, angle)

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

    def visualize_map(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap="gray_r", interpolation="nearest")
        plt.colorbar(label="Occupancy (0: Unknown, 0.5: Free, 1: Occupied)")
        plt.plot(self.position[0], self.position[1], "ro", markersize=10)
        plt.title("Environment Map")
        plt.xlabel("X coordinate (cm)")
        plt.ylabel("Y coordinate (cm)")
        plt.show()

    def run_mapping(self, duration=60):
        start_time = time.time()

        while time.time() - start_time < duration:
            distance = self.get_distance()
            if distance < 30:
                self.PWM.setMotorModel(0, 0, 0, 0)  # Stop before avoiding
                self.obstacle_avoidance()
            else:
                self.scan_environment()
                self.line_tracking()

            # Update position (simplified, assuming constant speed)
            self.position[0] += (
                cos(radians(self.angle)) * 5
            )  # Adjust the constant as needed
            self.position[1] += sin(radians(self.angle)) * 5

            # Periodically visualize the map
            if int(time.time() - start_time) % 10 == 0:
                self.visualize_map()

        self.visualize_map()
        print(f"Mapping completed.")
        print(
            f"Percentage of explored area: {np.sum(self.map > 0) / self.map.size * 100:.2f}%"
        )
        print(f"Number of detected obstacles: {np.sum(self.map == 1)}")

    def cleanup(self):
        self.PWM.setMotorModel(0, 0, 0, 0)
        self.led.colorWipe(self.led.strip, Color(0, 0, 0), 10)
        GPIO.cleanup()


# Main program logic follows:
if __name__ == "__main__":
    car = AdvancedMapping()
    print("Program is starting ... ")
    try:
        car.run_mapping(duration=120)  # Run mapping for 2 minutes
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        car.cleanup()
