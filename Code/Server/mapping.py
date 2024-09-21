import numpy as np
import math
import time
import matplotlib.pyplot as plt
from queue import PriorityQueue
import os
from datetime import datetime
import shutil


class Mapping:
    def __init__(self, map_size=200, resolution=1):
        self.map_size = map_size
        self.resolution = resolution
        self.true_map = np.zeros(
            (map_size, map_size), dtype=int
        )  # The true map with all obstacles
        self.known_map = np.zeros(
            (map_size, map_size), dtype=int
        )  # The map known to the car
        self.car_width = 25  # cm
        self.car_height = 20  # cm
        self.car_position = (map_size // 2, self.car_height // 2)
        self.car_orientation = 90  # 90 degrees is facing up (north)
        self.servo_angle = 90  # Initialize servo angle
        self.goal = (map_size - 50, map_size - 50)  # Static goal
        self.update_interval = 5  # Update map every 5 steps
        self.additional_padding = 5  # 5 cm additional padding

        # Create output folder for visualizations
        self.output_folder = "mappings/Routing"
        if os.path.exists(self.output_folder):
            shutil.rmtree(self.output_folder)
        os.makedirs(self.output_folder)

        # Create mock environment with some obstacles
        self.create_mock_environment()

        # Initialize mock components
        self.ultrasonic = MockUltrasonic(self)
        self.servo = MockServo(self)
        self.motor = MockMotor()

    def create_mock_environment(self):
        # Create static obstacles in the true map
        obstacles = [
            (50, 50, 40, 40),
            (120, 100, 20, 20),
        ]
        for obstacle in obstacles:
            self.add_rectangle_obstacle(*obstacle)

    def add_rectangle_obstacle(self, x, y, width, height):
        # Add obstacle with clearance to the true map
        clearance = max(self.car_width, self.car_height) // 2 + self.additional_padding
        x_start = max(0, x - clearance)
        y_start = max(0, y - clearance)
        x_end = min(self.map_size, x + width + clearance)
        y_end = min(self.map_size, y + height + clearance)
        self.true_map[y_start:y_end, x_start:x_end] = 1

    def heuristic(self, a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    def get_neighbors(self, current):
        x, y = current
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_x, next_y = x + dx, y + dy
            if self.is_valid_position((next_x, next_y)):
                neighbors.append((next_x, next_y))
        return neighbors

    def is_valid_position(self, position):
        x, y = position
        half_width = self.car_width // 2
        half_height = self.car_height // 2

        # Check if the entire car footprint is within the map and not on a detected obstacle
        for dx in range(-half_width, half_width + 1):
            for dy in range(-half_height, half_height + 1):
                check_x, check_y = x + dx, y + dy
                if (
                    check_x < 0
                    or check_x >= self.map_size
                    or check_y < 0
                    or check_y >= self.map_size
                    or self.known_map[check_y, check_x]
                    == 2  # Only consider detected obstacles
                ):
                    return False
        return True

    def a_star_search(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current

        # Reconstruct path
        path = []
        current = goal
        while current != start:
            if current not in came_from:
                return None  # No path found
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def move_car(self, new_position):
        if not self.is_valid_position(new_position):
            print(f"Cannot move to {new_position}, obstacle detected or out of bounds.")
            return False

        # Update car orientation based on movement
        dx = new_position[0] - self.car_position[0]
        dy = new_position[1] - self.car_position[1]
        if dx != 0 or dy != 0:
            self.car_orientation = math.degrees(math.atan2(dy, dx))

        self.car_position = new_position
        print(f"Car moved to {new_position}")
        return True

    def update_map_during_movement(self):
        print("Updating map during movement...")
        self.scan_environment()

    def navigate_to_goal(self):
        # Perform initial scan and map update
        print("Performing initial scan and map update...")
        self.scan_environment()
        self.visualize_map(self.known_map, "Initial Scan")

        steps_since_update = 0
        step_count = 0
        while self.car_position != self.goal:
            print(f"Current position: {self.car_position}, Goal: {self.goal}")
            path = self.a_star_search(self.car_position, self.goal)

            if path is None or len(path) < 2:
                print("No valid path to goal.")
                break

            # Visualize the current state with the planned path
            self.visualize_map(
                self.known_map,
                f"Step {step_count}: Planned path from {self.car_position}",
                path,
            )
            step_count += 1

            # Move along the path for a few steps
            steps_to_take = min(self.update_interval, len(path) - 1)
            for i in range(1, steps_to_take + 1):
                if not self.move_car(path[i]):
                    break  # Stop if we can't move to the next position
                steps_since_update += 1

                if steps_since_update >= self.update_interval:
                    self.update_map_during_movement()
                    steps_since_update = 0
                    break  # Replan after updating the map

            if self.car_position == self.goal:
                print("Goal reached!")
                self.visualize_map(
                    self.known_map, f"Step {step_count}: Goal Reached", None
                )
                break

    def scan_environment(self):
        print("Starting environment scan...")
        # Assuming this centers the servo vertically
        self.servo.setServoPwm("1", 90)

        scan_angles = range(30, 151, 6)  # Scan from 30 to 150 degrees in 6-degree steps
        scan_results = {}

        for angle in scan_angles:
            self.servo.setServoPwm("0", angle)
            distance = self.ultrasonic.get_distance()
            print(f"Scan at angle {angle}: Distance {distance} cm")
            scan_results[angle] = distance
            self.update_map(distance, angle)

        # Improved interpolation using step function approach
        for angle in range(30, 151):
            if angle not in scan_angles:
                # Find the two nearest scanned angles
                lower_angle = max([a for a in scan_angles if a <= angle])
                upper_angle = min([a for a in scan_angles if a >= angle])

                # Use the minimum distance of the two nearest scanned angles
                interpolated_distance = min(
                    scan_results[lower_angle], scan_results[upper_angle]
                )

                print(
                    f"Interpolated scan at angle {angle}: Distance {interpolated_distance} cm"
                )
                self.update_map(interpolated_distance, angle)

        print("Environment scan completed.")

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

    def visualize_map(self, map, title="Map Visualization", path=None):
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
        dx = 15 * math.cos(car_angle)
        dy = 15 * math.sin(car_angle)

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
            car_x, car_y, dx, dy, head_width=5, head_length=10, fc="blue", ec="blue"
        )

        # Plot goal position
        goal_x, goal_y = self.goal
        plt.plot(goal_x, goal_y, "g*", markersize=15)

        # Plot path if provided
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, color="blue", linewidth=2, linestyle="--")

        # Add legend
        legend_elements = [
            plt.Rectangle(
                (0, 0), 1, 1, facecolor="white", edgecolor="black", label="Empty"
            ),
            plt.Rectangle((0, 0), 1, 1, facecolor="red", label="Detected Obstacle"),
            plt.Rectangle(
                (0, 0), 1, 1, facecolor="gray", alpha=0.5, label="True Obstacle"
            ),
            plt.Line2D(
                [0], [0], color="blue", lw=2, linestyle="--", label="Planned Path"
            ),
            plt.Rectangle((0, 0), 1, 1, facecolor="blue", label="Car"),
            plt.plot([], [], "g*", markersize=15, label="Goal")[0],
        ]
        plt.legend(handles=legend_elements, loc="upper left", bbox_to_anchor=(1, 1))

        plt.tight_layout()

        # Save the figure instead of showing it
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.output_folder}/map_{timestamp}.png"
        plt.savefig(filename, bbox_inches="tight")
        plt.close()
        print(f"Map saved as {filename}")


class MockUltrasonic:
    def __init__(self, environment):
        self.environment = environment

    def get_distance(self):
        x, y = self.environment.car_position
        angle = self.environment.car_orientation + self.environment.servo_angle - 90
        rad_angle = math.radians(angle)

        for distance in range(1, 300):
            check_x = int(x + distance * math.cos(rad_angle))
            check_y = int(y + distance * math.sin(rad_angle))

            if (
                0 <= check_x < self.environment.map_size
                and 0 <= check_y < self.environment.map_size
            ):
                if self.environment.true_map[check_y, check_x] != 0:
                    return distance
            else:
                return distance

        return 300


class MockServo:
    def __init__(self, environment):
        self.environment = environment

    def setServoPwm(self, axis, angle):
        self.environment.servo_angle = angle
        print(f"Servo rotated to {angle} degrees")


class MockMotor:
    def __init__(self):
        pass

    def setMotorModel(self, *args):
        print(f"Motor set to: {args}")


# Example usage
if __name__ == "__main__":
    mapper = Mapping()
    print("Visualizing initial map configuration...")
    mapper.visualize_map(mapper.true_map, "Initial Map Configuration")

    print("\nNavigating to goal...")
    mapper.navigate_to_goal()

    print("\nVisualization complete. Check the 'mappings' folder for saved images.")
