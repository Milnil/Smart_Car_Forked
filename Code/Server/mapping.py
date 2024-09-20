import numpy as np
import matplotlib.pyplot as plt
import math
import time
import logging
from Motor import Motor
from servo import Servo
from Ultrasonic import Ultrasonic
import RPi.GPIO as GPIO


class ContinuousMappingAndNavigation:
    def __init__(
        self,
        map_size=(200, 200),
        initial_position=(100, 10),
        initial_angle=90,
        goal=(160, 160),
    ):
        # Set up logging
        logging.basicConfig(
            filename="navigation.log",
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
        )
        self.logger = logging.getLogger()

        # Initialize mapping parameters
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.max_sensor_range = 300  # Maximum range of ultrasonic sensor in cm
        self.goal = goal

        # Car dimensions (in cm)
        self.car_length = 25
        self.car_width = 20

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

        # Set initial servo position (10 degrees up)
        self.servo.setServoPwm(
            "0", 100
        )  # Assuming 90 is horizontal, 100 should be 10 degrees up
        time.sleep(0.5)  # Allow time for servo to move

        self.logger.info(
            "Initialization complete. Starting position: %s, Goal: %s",
            self.position,
            self.goal,
        )

    def get_ultrasonic_distance(self):
        """Get distance reading from ultrasonic sensor"""
        distance = self.ultrasonic.get_distance()
        self.logger.debug("Ultrasonic distance reading: %s cm", distance)
        return distance

    def rotate_servo(self, angle):
        """Rotate servo to specified angle"""
        self.servo.setServoPwm("0", angle + 10)  # Add 10 to keep it slightly elevated
        time.sleep(0.1)  # Allow time for servo to move
        self.logger.debug("Servo rotated to angle: %s", angle)

    def scan_environment(self):
        """Perform a 150-degree scan of the environment"""
        self.logger.info("Starting environment scan")
        for angle in range(15, 166, 5):  # Scan from 15 to 165 degrees in 5-degree steps
            self.rotate_servo(angle)
            distance = self.get_ultrasonic_distance()
            self.update_map(distance, angle)
        self.logger.info("Environment scan complete")

    def update_map(self, distance, angle):
        """Update the map based on sensor reading, considering car as a rectangle"""
        rad_angle = math.radians(
            self.angle + angle - 90
        )  # Adjust for car's orientation

        # Calculate the four corners of the car
        corners = [
            (
                self.position[0] - self.car_width / 2,
                self.position[1] - self.car_length / 2,
            ),
            (
                self.position[0] + self.car_width / 2,
                self.position[1] - self.car_length / 2,
            ),
            (
                self.position[0] - self.car_width / 2,
                self.position[1] + self.car_length / 2,
            ),
            (
                self.position[0] + self.car_width / 2,
                self.position[1] + self.car_length / 2,
            ),
        ]

        # Mark the car's position as occupied
        for corner in corners:
            x, y = int(corner[0]), int(corner[1])
            if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
                self.map[y, x] = 0.5  # 0.5 indicates car's position

        # Mark the detected obstacle
        x = int(self.position[0] + distance * math.cos(rad_angle))
        y = int(self.position[1] + distance * math.sin(rad_angle))

        if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
            self.map[y, x] = 1  # Mark as obstacle

            # Mark cells along the line as free space
            for i in range(1, int(distance)):
                ix = int(self.position[0] + i * math.cos(rad_angle))
                iy = int(self.position[1] + i * math.sin(rad_angle))
                if 0 <= ix < self.map_size[0] and 0 <= iy < self.map_size[1]:
                    self.map[iy, ix] = 0.2  # 0.2 indicates free space

        self.logger.debug("Map updated. Obstacle detected at (%s, %s)", x, y)

    def move(self, speed, duration):
        """Move the car forward at specified speed for a duration"""
        self.motor.setMotorModel(speed, speed, speed, speed)
        start_time = time.time()

        while time.time() - start_time < duration:
            self.update_position()
            time.sleep(0.1)

        self.motor.setMotorModel(0, 0, 0, 0)  # Stop motors
        self.logger.info("Moved for %s seconds at speed %s", duration, speed)

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
        self.logger.debug("Position updated to: %s", self.position)

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
        self.logger.info("Rotated by %s degrees. New angle: %s", angle, self.angle)

    def visualize_map(self):
        """Visualize the current map"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap="gray_r", interpolation="nearest")
        plt.colorbar(label="Occupancy (0: Unknown, 0.2: Free, 0.5: Car, 1: Occupied)")
        plt.plot(self.position[0], self.position[1], "ro", markersize=10)
        plt.plot(self.goal[0], self.goal[1], "go", markersize=10)
        plt.title("Environment Map")
        plt.xlabel("X coordinate (cm)")
        plt.ylabel("Y coordinate (cm)")
        plt.savefig(f"map_{time.time()}.png")
        plt.close()
        self.logger.info("Map visualized and saved")

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
                self.logger.info("Obstacle detected. Choosing to turn left.")
                return "rotate", -45
            else:
                self.logger.info("Obstacle detected. Choosing to turn right.")
                return "rotate", 45
        else:
            # If no immediate obstacle, adjust angle towards goal
            if abs(angle_diff) > 10:
                self.logger.info(
                    "Adjusting angle towards goal by %s degrees.", angle_diff
                )
                return "rotate", angle_diff
            else:
                move_distance = min(
                    front_distance, 30
                )  # Move forward, but not more than 30 cm at a time
                self.logger.info("Moving forward by %s cm.", move_distance)
                return "move", move_distance

    def run_continuous_mapping_and_navigation(self, duration=300):
        """Continuously map the environment and navigate towards the goal"""
        start_time = time.time()
        self.logger.info(
            "Starting continuous mapping and navigation. Duration: %s seconds", duration
        )

        while time.time() - start_time < duration:
            # Scan environment
            self.scan_environment()

            # Visualize current map
            self.visualize_map()

            # Check if goal is reached
            distance_to_goal = math.sqrt(
                (self.position[0] - self.goal[0]) ** 2
                + (self.position[1] - self.goal[1]) ** 2
            )
            if distance_to_goal < 10:  # Within 10 cm of goal
                self.logger.info("Goal reached! Final position: %s", self.position)
                break

            # Get best move
            action, value = self.get_best_move()

            # Execute move
            if action == "rotate":
                self.rotate(value)
            elif action == "move":
                self.move(1000, value / 50)  # Adjust speed and time as needed

            self.logger.info(
                "Current position: %s, Distance to goal: %s cm",
                self.position,
                distance_to_goal,
            )

        self.logger.info(
            "Navigation completed. Total distance traveled: %.2f cm",
            self.distance_traveled,
        )
        self.logger.info(
            "Percentage of explored area: %.2f%%",
            np.sum(self.map > 0) / self.map.size * 100,
        )
        self.logger.info("Number of detected obstacles: %d", np.sum(self.map == 1))


# Main execution
if __name__ == "__main__":
    try:
        nav = ContinuousMappingAndNavigation()
        nav.run_continuous_mapping_and_navigation(duration=300)  # Run for 5 minutes
    except KeyboardInterrupt:
        nav.logger.warning("Process interrupted by user")
    except Exception as e:
        nav.logger.error("An error occurred: %s", str(e), exc_info=True)
    finally:
        # Clean up
        GPIO.cleanup()
        nav.motor.setMotorModel(0, 0, 0, 0)
        nav.servo.setServoPwm("0", 90)
        nav.logger.info("Navigation process ended. Cleanup complete.")
