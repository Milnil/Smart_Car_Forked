import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians
import time
from Motor import *
from servo import *
from Ultrasonic import *
from PCA9685 import PCA9685
import RPi.GPIO as GPIO


class AdvancedMapping:
    def __init__(
        self, map_size=(200, 200), initial_position=(100, 100), initial_angle=0
    ):
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.max_sensor_range = 300  # Maximum range of the ultrasonic sensor in cm

        # Initialize car components
        self.PWM = Motor()
        self.pwm_S = Servo()
        self.ultrasonic = Ultrasonic()

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Photo-interrupter pins for velocity sensing
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

        self.last_time = time.time()
        self.distance_traveled = 0

    def get_ultrasonic_distance(self):
        return self.ultrasonic.get_distance()

    def rotate_servo(self, angle):
        self.pwm_S.setServoPwm("0", angle)
        time.sleep(0.1)  # Allow time for the servo to move

    def scan_environment(self):
        for angle in range(
            30, 151, 15
        ):  # Scan from 30 to 150 degrees in 15-degree steps
            self.rotate_servo(angle)
            distance = self.get_ultrasonic_distance()
            self.update_map(distance, angle)

    def update_map(self, distance, angle):
        rad_angle = radians(self.angle + angle - 90)  # Adjust for car's orientation
        x = int(self.position[0] + distance * cos(rad_angle))
        y = int(self.position[1] + distance * sin(rad_angle))

        if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
            self.map[y, x] = 1  # Mark as obstacle

            # Mark cells along the line as free space
            for i in range(1, int(distance)):
                ix = int(self.position[0] + i * cos(rad_angle))
                iy = int(self.position[1] + i * sin(rad_angle))
                if 0 <= ix < self.map_size[0] and 0 <= iy < self.map_size[1]:
                    self.map[iy, ix] = 0.5  # 0.5 indicates free space

    def move(self, speed, duration):
        self.PWM.setMotorModel(speed, speed, speed, speed)
        start_time = time.time()

        while time.time() - start_time < duration:
            # Update position based on velocity sensing
            self.update_position()
            time.sleep(0.1)

        self.PWM.setMotorModel(0, 0, 0, 0)  # Stop motors

    def update_position(self):
        current_time = time.time()
        dt = current_time - self.last_time

        # Simple velocity estimation using photo-interrupters
        left_speed = GPIO.input(self.IR01)
        right_speed = GPIO.input(self.IR03)

        avg_speed = (left_speed + right_speed) / 2
        distance = avg_speed * dt * 10  # Adjust this factor based on your wheel size

        self.distance_traveled += distance

        # Update position
        self.position[0] += distance * cos(radians(self.angle))
        self.position[1] += distance * sin(radians(self.angle))

        self.last_time = current_time

    def rotate(self, angle):
        # Implement rotation logic
        if angle > 0:
            self.PWM.setMotorModel(-1500, -1500, 2000, 2000)
        else:
            self.PWM.setMotorModel(2000, 2000, -1500, -1500)

        ### TODO
        time.sleep(abs(angle) / 90)  # Adjust this factor for accurate rotation
        self.PWM.setMotorModel(0, 0, 0, 0)

        self.angle += angle
        self.angle %= 360

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
            self.scan_environment()

            # Move forward if no obstacle
            if self.get_ultrasonic_distance() > 30:
                self.move(500, 1)  # Move forward for 1 second
            else:
                # Rotate to find clear path
                self.rotate(45)

            # Periodically visualize the map
            if int(time.time() - start_time) % 10 == 0:
                self.visualize_map()

        self.visualize_map()
        print(
            f"Mapping completed. Total distance traveled: {self.distance_traveled:.2f} cm"
        )
        print(
            f"Percentage of explored area: {np.sum(self.map > 0) / self.map.size * 100:.2f}%"
        )
        print(f"Number of detected obstacles: {np.sum(self.map == 1)}")


# Main execution
if __name__ == "__main__":
    try:
        mapping = AdvancedMapping()
        mapping.run_mapping(duration=120)  # Run mapping for 2 minutes
    except KeyboardInterrupt:
        print("Mapping interrupted by user")
    finally:
        # Clean up
        GPIO.cleanup()
        mapping.PWM.setMotorModel(0, 0, 0, 0)
        mapping.pwm_S.setServoPwm("0", 90)
