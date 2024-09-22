import numpy as np
import math
import time
import matplotlib.pyplot as plt
import os
from datetime import datetime
import shutil
from Motor import *
from Ultrasonic import *
from servo import *
from ObjectDetector import *


class Mapping:
    def __init__(self, map_size=400, resolution=4):
        self.map_size = map_size
        self.resolution = resolution
        self.true_map = np.zeros((map_size, map_size), dtype=int)
        self.known_map = np.zeros((map_size, map_size), dtype=int)
        self.car_width = 20
        self.car_height = 25
        self.car_position = (map_size // 2, self.car_height // 2)
        self.car_orientation = 90  # 90 degrees is facing up (north)
        self.servo_angle = 90  # Initialize servo angle
        self.goal = (250, 250)  # (horizontal), (vertical)
        self.additional_padding = 5
        self.detector = ObjectDetector()
        self.detected_objects = set()
        self.object_cooldown = 30  # Number of steps before re-detecting the same object
        self.object_cooldown_counter = {}
        self.detection_range = (
            15  # Distance in cm within which to perform object detection
        )

        # Create output folder for visualizations
        self.output_folder = "mappings/Routing"
        if os.path.exists(self.output_folder):
            shutil.rmtree(self.output_folder)
        os.makedirs(self.output_folder)

        # Initialize components
        self.ultrasonic = Ultrasonic()
        self.servo = Servo()
        self.motor = Motor()

    def move_car(self, distance):
        # Calculate the movement in x and y directions
        dx = distance * math.cos(math.radians(self.car_orientation))
        dy = distance * math.sin(math.radians(self.car_orientation))

        new_x = int(self.car_position[0] + dx)
        new_y = int(self.car_position[1] + dy)

        # Check if the new position is valid
        if self.is_valid_position((new_x, new_y)):
            # Move forward
            self.motor.setMotorModel(600, 600, 600, 600)
            time.sleep(distance * 0.1)  # Adjust this factor based on your car's speed
            self.motor.setMotorModel(0, 0, 0, 0)  # Stop movement

            # Update car position
            self.car_position = (new_x, new_y)
            print(f"Car moved to approximately {self.car_position}")
            return True
        else:
            print(
                f"Cannot move to {(new_x, new_y)}, obstacle detected or out of bounds."
            )
            return False

    def is_valid_position(self, position):
        x, y = position
        half_width = self.car_width // 2
        half_height = self.car_height // 2

        for dx in range(-half_width, half_width + 1):
            for dy in range(-half_height, half_height + 1):
                check_x, check_y = x + dx, y + dy
                if (
                    check_x < 0
                    or check_x >= self.map_size
                    or check_y < 0
                    or check_y >= self.map_size
                    or self.known_map[check_y, check_x] == 2
                ):
                    return False
        return True

    def scan_environment(self):
        print("Starting environment scan...")
        self.servo.setServoPwm("1", 110)  # Assuming this centers the servo vertically

        scan_angles = range(
            30, 130, 10
        )  # Scan from 30 to 150 degrees in 10-degree steps
        scan_results = {}

        for angle in scan_angles:
            self.servo.setServoPwm("0", angle)
            time.sleep(0.2)
            distance = self.ultrasonic.get_distance()

            if distance == 0:
                continue

            scan_results[angle] = distance
            self.update_map(distance, angle)

            if distance <= self.detection_range:
                self.perform_object_detection(angle)

            print(f"Scan at angle {angle}: Distance {distance} cm")

        # Improved interpolation using step function approach
        for angle in range(30, 130):
            if angle not in scan_angles:
                try:
                    lower_angle = max([a for a in scan_angles if a <= angle])
                    upper_angle = min([a for a in scan_angles if a >= angle])
                    interpolated_distance = min(
                        scan_results[lower_angle], scan_results[upper_angle]
                    )
                    self.update_map(interpolated_distance, angle)
                except:
                    pass

        print("Environment scan completed.")
        return scan_results

    def determine_safe_distance(self, scan_results):
        forward_distance = scan_results.get(
            90, float("inf")
        )  # Distance directly in front
        left_distance = scan_results.get(60, float("inf"))  # Distance to the left
        right_distance = scan_results.get(120, float("inf"))  # Distance to the right

        # Consider the minimum distance from these three directions
        min_distance = min(forward_distance, left_distance, right_distance)

        # Calculate a safe distance to move (e.g., 2/3 of the minimum distance)
        safe_distance = int(min_distance * 2 / 3)

        # Limit the safe distance to a maximum value (e.g., 50 cm)
        return min(safe_distance, 50)

    def turn_towards_goal(self):
        goal_x, goal_y = self.goal
        car_x, car_y = self.car_position

        # Calculate the angle to the goal
        dx = goal_x - car_x
        dy = goal_y - car_y
        angle_to_goal = math.degrees(math.atan2(dy, dx)) % 360

        # Calculate the difference between current orientation and angle to goal
        angle_diff = (angle_to_goal - self.car_orientation) % 360
        if angle_diff > 180:
            angle_diff -= 360

        # Determine turn direction and amount
        if abs(angle_diff) > 10:  # Only turn if the difference is significant
            if angle_diff > 0:
                print("Turning right")
                self.motor.setMotorModel(2000, 2000, -2000, -2000)  # Turn right
            else:
                print("Turning left")
                self.motor.setMotorModel(-2000, -2000, 2000, 2000)  # Turn left

            # Calculate turn duration based on angle difference
            turn_duration = abs(angle_diff) / 90  # Adjust this factor as needed
            time.sleep(turn_duration)
            self.motor.setMotorModel(0, 0, 0, 0)  # Stop turning

            self.car_orientation = angle_to_goal
            print(f"New orientation: {self.car_orientation}")

    def navigate_to_goal(self):
        step_count = 0
        while True:
            print(
                f"Step {step_count}: Current position: {self.car_position}, Goal: {self.goal}"
            )

            # Turn towards the goal
            self.turn_towards_goal()

            # Scan the environment
            scan_results = self.scan_environment()

            # Determine safe distance to move
            safe_distance = self.determine_safe_distance(scan_results)

            # Visualize the current state
            self.visualize_map(self.known_map, f"Step {step_count}: Before moving")

            # Move the car
            if self.move_car(safe_distance):
                print(f"Moved {safe_distance} cm towards the goal")
            else:
                print("Unable to move, replanning...")
                continue

            # Check if goal is reached
            if self.is_goal_reached():
                print("Goal reached!")
                self.visualize_map(self.known_map, f"Step {step_count}: Goal Reached")
                break

            step_count += 1

    def is_goal_reached(self):
        return (
            math.dist(self.car_position, self.goal) < 20
        )  # Adjust threshold as needed

    def perform_object_detection(self, angle):
        detected = self.detector.detect_objects(1)
        new_objects = detected - self.detected_objects

        for obj in new_objects:
            if (
                obj not in self.object_cooldown_counter
                or self.object_cooldown_counter[obj] <= 0
            ):
                if obj in ["stop sign", "traffic light"]:
                    print(f"Detected {obj} at angle {angle}. Stopping for 5 seconds.")
                    self.motor.setMotorModel(0, 0, 0, 0)  # Stop the car
                    time.sleep(5)
                    self.detected_objects.add(obj)
                    self.object_cooldown_counter[obj] = self.object_cooldown

        # Decrease cooldown counters
        for obj in list(self.object_cooldown_counter.keys()):
            self.object_cooldown_counter[obj] -= 1
            if self.object_cooldown_counter[obj] <= 0:
                del self.object_cooldown_counter[obj]
                self.detected_objects.remove(obj)

    def update_map(self, distance, angle):
        x, y = self.car_position
        rad_angle = math.radians(self.car_orientation + angle - 90)

        for d in range(0, distance, self.resolution):
            cell_x = int(x + d * math.cos(rad_angle))
            cell_y = int(y + d * math.sin(rad_angle))
            if 0 <= cell_x < self.map_size and 0 <= cell_y < self.map_size:
                self.known_map[cell_y, cell_x] = 0  # Mark as empty
        end_x = int(x + distance * math.cos(rad_angle))
        end_y = int(y + distance * math.sin(rad_angle))
        if 0 <= end_x < self.map_size and 0 <= end_y < self.map_size:
            self.known_map[end_y, end_x] = 2  # Mark detected obstacle

    def visualize_map(self, map, title="Map Visualization"):
        plt.figure(figsize=(10, 10))

        # Create a base map with empty cells (white) and detected obstacles (red)
        plt.imshow(
            map,
            cmap=plt.cm.colors.ListedColormap(["white", "red"]),
            interpolation="nearest",
        )

        # Overlay true obstacles in gray
        true_obstacle_mask = self.true_map == 1
        plt.imshow(
            true_obstacle_mask,
            cmap=plt.cm.colors.ListedColormap(["none", "gray"]),
            alpha=0.5,
            interpolation="nearest",
        )

        plt.title(title)

        # Plot car position and orientation
        car_x, car_y = self.car_position
        car_angle = math.radians(self.car_orientation)
        dx = 7 * math.cos(car_angle)
        dy = 7 * math.sin(car_angle)

        car_rect = plt.Rectangle(
            (car_x - self.car_width // 2, car_y - self.car_height // 2),
            self.car_width,
            self.car_height,
            angle=self.car_orientation - 90,
            fill=False,
            color="blue",
        )
        plt.gca().add_patch(car_rect)
        plt.arrow(
            car_x, car_y, dx, dy, head_width=2, head_length=4, fc="blue", ec="blue"
        )

        # Plot goal position
        goal_x, goal_y = self.goal
        plt.plot(goal_x, goal_y, "g*", markersize=10)

        # Add legend
        legend_elements = [
            plt.Rectangle(
                (0, 0), 1, 1, facecolor="white", edgecolor="black", label="Empty"
            ),
            plt.Rectangle((0, 0), 1, 1, facecolor="red", label="Detected Obstacle"),
            plt.Rectangle(
                (0, 0), 1, 1, facecolor="gray", alpha=0.5, label="True Obstacle"
            ),
            plt.Rectangle((0, 0), 1, 1, facecolor="blue", label="Car"),
            plt.plot([], [], "g*", markersize=10, label="Goal")[0],
        ]
        plt.legend(handles=legend_elements, loc="upper left", bbox_to_anchor=(1, 1))

        plt.tight_layout()

        # Save the figure
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.output_folder}/map_{timestamp}.png"
        plt.savefig(filename, bbox_inches="tight")
        plt.close()
        print(f"Map saved as {filename}")


# Example usage
if __name__ == "__main__":
    mapper = Mapping()
    print("Visualizing initial map configuration...")
    mapper.visualize_map(mapper.true_map, "Initial Map Configuration")

    print("\nNavigating to goal...")
    mapper.navigate_to_goal()

    print("\nVisualization complete. Check the 'mappings' folder for saved images.")
