import numpy as np
import matplotlib.pyplot as plt
import math
import time
from Motor import Motor
from servo import Servo
from Ultrasonic import Ultrasonic
import RPi.GPIO as GPIO


class ContinuousMappingAndNavigation:
    def __init__(
        self,
        map_size=(100, 100),
        initial_position=(50, 0),
        initial_angle=90,
        goal=(80, 80),
    ):
        # Initialize mapping parameters
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.max_sensor_range = 300  # Maximum range of ultrasonic sensor in cm
        self.goal = goal

        # Initialize car components
        self.motor = Motor()
        self.servo = Servo()
        self.ultrasonic = Ultrasonic()

        # Initialize GPIO for line tracking sensors
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

        self.last_time = time.time()
        self.distance_traveled = 0

    def get_ultrasonic_distance(self):
        """Get distance reading from ultrasonic sensor"""
        return self.ultrasonic.get_distance()

    def rotate_servo(self, angle):
        """Rotate servo to specified angle"""
        self.servo.setServoPwm("0", angle)
        time.sleep(0.1)  # Allow time for servo to move

    def scan_environment(self):
        """Perform a 120-degree scan of the environment"""
        for angle in range(30, 151, 6):  # Scan from 30 to 150 degrees in 6-degree steps
            self.rotate_servo(angle)
            distance = self.get_ultrasonic_distance()
            self.update_map(distance, angle)

    def update_map(self, distance, angle):
        """Update the map based on sensor reading"""
        rad_angle = math.radians(
            self.angle + angle - 90
        )  # Adjust for car's orientation
        x = int(self.position[0] + distance * math.cos(rad_angle))
        y = int(self.position[1] + distance * math.sin(rad_angle))

        if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
            self.map[y, x] = 1  # Mark as obstacle

            # Mark cells along the line as free space
            for i in range(1, int(distance)):
                ix = int(self.position[0] + i * math.cos(rad_angle))
                iy = int(self.position[1] + i * math.sin(rad_angle))
                if 0 <= ix < self.map_size[0] and 0 <= iy < self.map_size[1]:
                    self.map[iy, ix] = 0.5  # 0.5 indicates free space

    def move(self, speed, duration):
        """Move the car forward at specified speed for a duration"""
        self.motor.setMotorModel(speed, speed, speed, speed)
        start_time = time.time()

        while time.time() - start_time < duration:
            self.update_position()
            time.sleep(0.1)

        self.motor.setMotorModel(0, 0, 0, 0)  # Stop motors

    def update_position(self):
        """Update the car's position based on line tracking sensors"""
        current_time = time.time()
        dt = current_time - self.last_time

        # Simple velocity estimation using line tracking sensors
        left_speed = GPIO.input(self.IR01)
        right_speed = GPIO.input(self.IR03)

        avg_speed = (left_speed + right_speed) / 2
        distance = avg_speed * dt * 10  # Adjust this factor based on your wheel size

        self.distance_traveled += distance

        # Update position
        self.position[0] += distance * math.cos(math.radians(self.angle))
        self.position[1] += distance * math.sin(math.radians(self.angle))

        self.last_time = current_time

    def rotate(self, angle):
        """Rotate the car by the specified angle"""
        if angle > 0:
            self.motor.setMotorModel(-1500, -1500, 2000, 2000)  # Turn right
        else:
            self.motor.setMotorModel(2000, 2000, -1500, -1500)  # Turn left

        time.sleep(abs(angle) / 90 * 0.7)  # Adjust this factor for accurate rotation
        self.motor.setMotorModel(0, 0, 0, 0)

        self.angle += angle
        self.angle %= 360

    def visualize_map(self):
        """Visualize the current map"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap="gray_r", interpolation="nearest")
        plt.colorbar(label="Occupancy (0: Unknown, 0.5: Free, 1: Occupied)")
        plt.plot(self.position[0], self.position[1], "ro", markersize=10)
        plt.plot(self.goal[0], self.goal[1], "go", markersize=10)
        plt.title("Environment Map")
        plt.xlabel("X coordinate (cm)")
        plt.ylabel("Y coordinate (cm)")
        plt.show(block=False)
        plt.pause(0.1)
        plt.close()

    def get_best_move(self):
        """Determine the best move based on current map and goal"""
        goal_angle = (
            math.degrees(
                math.atan2(
                    self.goal[1] - self.position[1], self.goal[0] - self.position[0]
                )
            )
            % 360
        )
        angle_diff = (goal_angle - self.angle) % 360
        if angle_diff > 180:
            angle_diff -= 360

        # Check if there's an obstacle in front
        front_distance = self.get_ultrasonic_distance()
        if front_distance < 30:
            # If obstacle, choose between left and right turn
            left_clear = (
                self.map[int(self.position[1]), max(0, int(self.position[0] - 10))] != 1
            )
            right_clear = (
                self.map[
                    int(self.position[1]),
                    min(self.map_size[0] - 1, int(self.position[0] + 10)),
                ]
                != 1
            )
            if left_clear and (not right_clear or angle_diff < 0):
                return "rotate", -45
            else:
                return "rotate", 45
        else:
            # If no immediate obstacle, adjust angle towards goal
            if abs(angle_diff) > 10:
                return "rotate", angle_diff
            else:
                return "move", min(
                    front_distance, 30
                )  # Move forward, but not more than 30 cm at a time

    def run_continuous_mapping_and_navigation(self, duration=180):
        """Continuously map the environment and navigate towards the goal"""
        start_time = time.time()

        while time.time() - start_time < duration:
            # Scan environment
            self.scan_environment()

            # Visualize current map
            self.visualize_map()

            # Check if goal is reached
            if (
                math.sqrt(
                    (self.position[0] - self.goal[0]) ** 2
                    + (self.position[1] - self.goal[1]) ** 2
                )
                < 5
            ):
                print("Goal reached!")
                break

            # Get best move
            action, value = self.get_best_move()

            # Execute move
            if action == "rotate":
                self.rotate(value)
            elif action == "move":
                self.move(1000, value / 50)  # Adjust speed and time as needed

        print(
            f"Navigation completed. Total distance traveled: {self.distance_traveled:.2f} cm"
        )
        print(
            f"Percentage of explored area: {np.sum(self.map > 0) / self.map.size * 100:.2f}%"
        )
        print(f"Number of detected obstacles: {np.sum(self.map == 1)}")


# Main execution
if __name__ == "__main__":
    try:
        nav = ContinuousMappingAndNavigation()
        nav.run_continuous_mapping_and_navigation(duration=300)  # Run for 5 minutes
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        # Clean up
        GPIO.cleanup()
        nav.motor.setMotorModel(0, 0, 0, 0)
        nav.servo.setServoPwm("0", 90)
