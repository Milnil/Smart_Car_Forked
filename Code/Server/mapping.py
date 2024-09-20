import numpy as np
import matplotlib.pyplot as plt
import math
import time
import logging
import heapq
from Motor import Motor
from servo import Servo
from Ultrasonic import Ultrasonic
import RPi.GPIO as GPIO
# Import for obstacle inflation
from scipy.ndimage import binary_dilation
# Placeholder import for TensorFlow model
# from tensorflow_model import TensorFlowModel


class ContinuousMappingAndNavigation:
    def __init__(
        self,
        map_size=(100, 100),
        initial_position=(50, 0),
        initial_angle=90,
        goal=(80, 0),
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
        
        # Placeholder for TensorFlow model
        # self.tensorflow_model = TensorFlowModel()

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

        # Map resolution (assuming 1 cm per cell)
        self.map_resolution = 1  # cm per cell

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
        self.servo.setServoPwm("1",110)
        for angle in range(30, 150, 10):  # Scan from 15 to 165 degrees in 5-degree steps
            self.rotate_servo(angle)
            distance = self.get_ultrasonic_distance()
            self.update_map(distance, angle)
            print(distance)
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
                    if self.map[iy, ix] == 0:
                        self.map[iy, ix] = 0.2  # 0.2 indicates free space

        self.logger.debug("Map updated. Obstacle detected at (%s, %s)", x, y)

    def move(self, speed, duration):
        """Move the car forward at specified speed for a duration"""
        self.motor.setMotorModel(speed, speed, speed, speed)
        start_time = time.time()

        while time.time() - start_time < duration:
            # Check for person detection
            if self.is_person_detected():
                self.logger.info("Person detected. Stopping until path is clear.")
                self.motor.setMotorModel(0, 0, 0, 0)  # Stop motors
                # Wait until person is no longer detected
                while self.is_person_detected():
                    time.sleep(0.1)
                self.logger.info("Person no longer detected. Resuming movement.")
                self.motor.setMotorModel(speed, speed, speed, speed)

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

        def is_collision(self, node):
                x,y = node
                x = int(x)
                y = int(y)
                


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
        plt.savefig(f"maps/map_{time.time()}.png")
        plt.close()
        self.logger.info("Map visualized and saved")

    
    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from came_from map."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    def a_star_search(self, start, goal):
        """Perform A* search to find the shortest path from start to goal."""
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        # Define movement directions: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if not (0 <= neighbor[0] < self.map_size[0] and 0 <= neighbor[1] < self.map_size[1]):
                    continue  # Skip out-of-bounds positions

                if self.inflated_map[neighbor[1], neighbor[0]] >= 0.5:
                    continue  # Skip occupied or inflated cells

                tentative_g_score = g_score[current] + 1  # Cost between adjacent nodes is 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def is_person_detected(self):
        """Check if a person is detected using TensorFlow object detection."""
        # Placeholder implementation
        # Replace with actual code that interfaces with TensorFlow model
        # Example:
        # image = self.capture_image()
        # detections = self.tensorflow_model.detect(image)
        # return any('person' in detection['class'] for detection in detections)

        # For now, we return False to proceed
        return False

    def get_best_move(self):
        """Determine the best move based on current map and goal using A* path planning."""
        # Inflate obstacles

        # Plan path using A*
        path = self.a_star_search(self.position, self.goal)

        if path is None or len(path) < 2:
            self.logger.warning("No path found to the goal.")
            return "rotate", 90  # Rotate to find a new path

        # Get the next position in the path
        next_position = path[1]  # path[0] is the current position

        # Calculate the required angle to move towards next_position
        dx = next_position[0] - self.position[0]
        dy = next_position[1] - self.position[1]
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        angle_diff = (target_angle - self.angle + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360

        if abs(angle_diff) > 5:
            self.logger.info(
                "Adjusting angle towards next path point by %.2f degrees.", angle_diff
            )
            return "rotate", angle_diff
        else:
            # Calculate distance to next position
            distance = math.hypot(dx, dy)
            move_distance = min(distance, 30)  # Move up to 30 cm
            self.logger.info("Moving forward by %.2f cm towards next path point.", move_distance)
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

            # Get best move
            action, value = self.get_best_move()

            # Execute move
            if action == "rotate":
                self.rotate(value)
            elif action == "move":
                self.move(1000, value / 50)  # Adjust speed and time as needed

            # After move, update distance to goal
            distance_to_goal_x = abs(self.position[0] - self.goal[0])
            distance_to_goal_y = abs(self.position[1] - self.goal[1])
            distance_to_goal = math.hypot(distance_to_goal_x, distance_to_goal_y)

            # Log current position and distance to goal
            self.logger.info(
                "Current position: %s, Distance to goal: %.2f cm",
                self.position,
                distance_to_goal,
            )

            # Check if goal is reached (within 5x5 radius)
            if distance_to_goal_x <= 5 and distance_to_goal_y <= 5:
                self.logger.info("Goal reached! Final position: %s", self.position)
                break

        self.logger.info(
            "Navigation completed. Total distance traveled: %.2f cm",
            self.distance_traveled,
        )
        self.logger.info(
            "Percentage of explored area: %.2f%%",
            np.sum(self.map > 0) / self.map.size * 100,
        )
        self.logger.info("Number of detected obstacles: %d", np.sum(self.map == 1))

    # Add any additional methods required for TensorFlow object detection
    # def capture_image(self):
    #     """Capture an image from the camera."""
    #     # Implement camera capture logic
    #     pass

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
        GPIO.cleanup()
        nav.motor.setMotorModel(0,0,0,0)
        nav.servo.setServoPwm("0",90)
        nav.logger.info("Navigation process ended. Cleanup complete.")
