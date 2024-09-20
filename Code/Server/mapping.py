import numpy as np
import matplotlib.pyplot as plt
import math
import time
from Motor import Motor
from servo import Servo
from Ultrasonic import Ultrasonic
import RPi.GPIO as GPIO


class AdvancedMappingAndNavigation:
    def __init__(self, map_size=(100, 100), initial_position=(50, 0), initial_angle=90):
        # Initialize mapping parameters
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.max_sensor_range = 300  # Maximum range of ultrasonic sensor in cm

        # Initialize car components
        self.motor = Motor()
        self.servo = Servo()
        self.servo.setServoPwm("1", 100)

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
        plt.title("Environment Map")
        plt.xlabel("X coordinate (cm)")
        plt.ylabel("Y coordinate (cm)")
        plt.show()

    def find_path(self, goal):
        """Find a path to the goal using a simple breadth-first search"""
        queue = [(int(self.position[0]), int(self.position[1]))]
        visited = set(queue)
        parent = {}

        while queue:
            current = queue.pop(0)
            if current == goal:
                break

            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_pos = (current[0] + dx, current[1] + dy)
                if (
                    0 <= next_pos[0] < self.map_size[0]
                    and 0 <= next_pos[1] < self.map_size[1]
                    and self.map[next_pos[1], next_pos[0]] != 1
                    and next_pos not in visited
                ):
                    queue.append(next_pos)
                    visited.add(next_pos)
                    parent[next_pos] = current

        if goal not in parent:
            return None

        path = []
        while goal != (int(self.position[0]), int(self.position[1])):
            path.append(goal)
            goal = parent[goal]
        path.reverse()
        return path

    def navigate_to_goal(self, goal):
        """Navigate to a goal position using the created map"""
        path = self.find_path(goal)
        if not path:
            print("No path found to goal")
            return

        for next_pos in path:
            # Calculate angle to next position
            dx = next_pos[0] - self.position[0]
            dy = next_pos[1] - self.position[1]
            target_angle = math.degrees(math.atan2(dy, dx)) % 360

            # Rotate to face the next position
            angle_diff = (target_angle - self.angle) % 360
            if angle_diff > 180:
                angle_diff -= 360
            self.rotate(angle_diff)

            # Move to the next position
            distance = math.sqrt(dx**2 + dy**2)
            self.move(500, distance / 50)  # Adjust speed and time as needed

            # Update position (in case move() didn't update it accurately)
            self.position = np.array(next_pos, dtype=float)

            # Scan environment after each move
            self.scan_environment()

    def run_mapping_and_navigation(self, duration=120):
        """Run the mapping process and navigate to a goal"""
        start_time = time.time()
        goal = (80, 80)  # Example goal position

        while time.time() - start_time < duration:
            self.scan_environment()

            # Try to navigate to the goal every 10 seconds
            if int(time.time() - start_time) % 10 == 0:
                print("Attempting to navigate to goal...")
                self.navigate_to_goal(goal)

            # # Move forward if no obstacle
            # if self.get_ultrasonic_distance() > 30:
            #     self.move(1000, 0.5)  # Move forward for 0.5 seconds
            # else:
            #     # Rotate to find clear path
            #     self.rotate(45)

            # Periodically visualize the map
            if int(time.time() - start_time) % 30 == 0:
                self.visualize_map()

        self.visualize_map()
        print(
            f"Mapping and navigation completed. Total distance traveled: {self.distance_traveled:.2f} cm"
        )
        print(
            f"Percentage of explored area: {np.sum(self.map > 0) / self.map.size * 100:.2f}%"
        )
        print(f"Number of detected obstacles: {np.sum(self.map == 1)}")


# Main execution
if __name__ == "__main__":
    try:
        mapping_and_navigation = AdvancedMappingAndNavigation()
        mapping_and_navigation.run_mapping_and_navigation(
            duration=180
        )  # Run for 3 minutes
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        # Clean up
        GPIO.cleanup()
        mapping_and_navigation.motor.setMotorModel(0, 0, 0, 0)
        mapping_and_navigation.servo.setServoPwm("0", 90)
