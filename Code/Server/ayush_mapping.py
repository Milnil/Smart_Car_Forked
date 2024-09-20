import numpy as np
import math
import time
import matplotlib.pyplot as plt
from Motor import Motor
from servo import Servo
from Ultrasonic import Ultrasonic


class ContinuousMappingAndNavigation:
    def __init__(
        self,
        map_size=(100, 100),
        initial_position=(0, 50),
        initial_angle=90,
        goal=(20, 50),
    ):
        self.map_size = map_size
        self.position = np.array(initial_position, dtype=float)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.goal = np.array(goal, dtype=float)
        self.max_sensor_range = 300  # Maximum range of ultrasonic sensor in cm

        self.motor = Motor()
        self.servo = Servo()
        self.ultrasonic = Ultrasonic()

        self.last_map_time = time.time()

    def get_ultrasonic_distance(self):
        return self.ultrasonic.get_distance()

    def rotate_servo(self, angle, axis=0):
        self.servo.setServoPwm(str(axis), angle)
        time.sleep(0.1)  # Allow time for servo to move

    def scan_environment(self):
        self.rotate_servo(100, 1)
        for angle in range(30, 151, 10):  # Scan 120 degrees in 10-degree steps
            self.rotate_servo(angle)
            distance = self.get_ultrasonic_distance()
            print(f"Angle: {angle}, Distance: {distance} cm")
            self.update_map(distance, angle)

    def update_map(self, distance, angle):
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
                    self.map[iy, ix] = 0  
                    # 0 indicates free space

    def move(self, speed, duration):
        self.motor.setMotorModel(speed, speed, speed, speed)
        time.sleep(duration)
        self.motor.setMotorModel(0, 0, 0, 0)  # Stop motors

        # Update position (simplified, assumes perfect movement)
        distance = (
            speed * duration / 100
        )  # Adjust this factor based on your motor's characteristics
        self.position[0] += distance * math.cos(math.radians(self.angle))
        self.position[1] += distance * math.sin(math.radians(self.angle))

        # Ensure position stays within map bounds
        self.position = np.clip(
            self.position, [0, 0], [self.map_size[0] - 1, self.map_size[1] - 1]
        )

    def rotate(self, angle):
        if angle > 0:
            self.motor.setMotorModel(-1500, -1500, 2000, 2000)  # Turn right
        else:
            self.motor.setMotorModel(2000, 2000, -1500, -1500)  # Turn left

        time.sleep(abs(angle) / 90 * 0.7)  # Adjust this factor for accurate rotation
        self.motor.setMotorModel(0, 0, 0, 0)

        self.angle += angle
        self.angle %= 360

    def find_best_move(self):
        # Simplified navigation: move towards the goal if possible, otherwise rotate
        goal_angle = math.degrees(
            math.atan2(self.goal[1] - self.position[1], self.goal[0] - self.position[0])
        )
        angle_diff = (goal_angle - self.angle + 180) % 360 - 180

        if abs(angle_diff) > 10:  # If not facing the goal, rotate
            return ("rotate", angle_diff)
        else:
            # Check if there's an obstacle in front
            front_distance = self.get_ultrasonic_distance()
            if front_distance > 20:  # If no close obstacle, move forward
                return (
                    "move",
                    min(front_distance - 10, 30),
                )  # Move up to 30 cm or just before the obstacle
            else:
                return ("rotate", 45)  # If obstacle in front, rotate 45 degrees

    def visualize_map(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap="binary")
        plt.colorbar(label="Occupancy (0: Free, 1: Occupied)")
        plt.title(f'Environment Map at {time.strftime("%Y-%m-%d %H:%M:%S")}')
        plt.xlabel("X coordinate (cm)")
        plt.ylabel("Y coordinate (cm)")

        # Plot the car's position and orientation
        plt.plot(self.position[0], self.position[1], "ro", markersize=10)
        orientation_line = plt.Line2D(
            (
                self.position[0],
                self.position[0] + 5 * math.cos(math.radians(self.angle)),
            ),
            (
                self.position[1],
                self.position[1] + 5 * math.sin(math.radians(self.angle)),
            ),
            lw=2,
            color="red",
            zorder=3,
        )
        plt.gca().add_line(orientation_line)

        # Plot the goal
        plt.plot(self.goal[0], self.goal[1], "g*", markersize=10)

        # Save the figure
        plt.savefig(f'maps/map_{time.strftime("%m%d_%H%M%S")}.png')
        plt.close()

    def run_continuous_mapping_and_navigation(self, duration=300):
        start_time = time.time()
        while time.time() - start_time < duration:
            # Scan environment
            self.scan_environment()

            self.visualize_map()
            self.last_map_time = time.time()

            # Get best move
            action, value = self.find_best_move()

            # Execute move
            if action == "rotate":
                self.rotate(value)
            elif action == "move":
                self.move(1000, value / 50)  # Adjust speed and time as needed

            # Check if goal is reached
            if np.linalg.norm(self.position - self.goal) < 10:  # Within 10 cm of goal
                print("Goal reached!")
                self.visualize_map()  # Final map visualization
                break

            print(f"Current position: {self.position}, Angle: {self.angle}")
            print(
                f"Distance to goal: {np.linalg.norm(self.position - self.goal):.2f} cm"
            )


# Main execution
if __name__ == "__main__":
    try:
        nav = ContinuousMappingAndNavigation()
        nav.run_continuous_mapping_and_navigation(duration=300)  # Run for 5 minutes
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        # Clean up
        nav.motor.setMotorModel(0, 0, 0, 0)
        nav.servo.setServoPwm("0", 90)
        nav.visualize_map()  # Final map visualization
