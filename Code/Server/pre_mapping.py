import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians


class AutonomousCar:
    def __init__(self, map_size=(100, 100), initial_position=(50, 50), initial_angle=0):
        self.map_size = map_size
        self.position = np.array(initial_position)
        self.angle = initial_angle
        self.map = np.zeros(map_size)
        self.max_sensor_range = 50

    def simulate_ultrasonic_reading(self):
        # Simulate a sensor reading
        return np.random.uniform(10, self.max_sensor_range)

    def rotate_servo(self, angle):
        # Simulate rotating the servo to a specific angle
        self.angle = angle

    def scan_environment(self):
        # Scan from 0 to 180 degrees in 15-degree steps
        for angle in range(0, 180, 15):
            self.rotate_servo(angle)
            distance = self.simulate_ultrasonic_reading()
            self.update_map(distance, angle)

    def update_map(self, distance, angle):
        # Convert polar coordinates to cartesian
        rad_angle = radians(angle)
        x = int(self.position[0] + distance * cos(rad_angle))
        y = int(self.position[1] + distance * sin(rad_angle))

        # Check if the point is within the map bounds
        if 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]:
            # Mark the cell as occupied
            self.map[y, x] = 1

            # Simple interpolation: mark cells along the line as free
            for i in range(1, int(distance)):
                ix = int(self.position[0] + i * cos(rad_angle))
                iy = int(self.position[1] + i * sin(rad_angle))
                if 0 <= ix < self.map_size[0] and 0 <= iy < self.map_size[1]:
                    self.map[iy, ix] = 0.5  # 0.5 indicates free space

    def move(self, distance, angle):
        # Simulate moving the car
        rad_angle = radians(angle)
        new_x = self.position[0] + distance * cos(rad_angle)
        new_y = self.position[1] + distance * sin(rad_angle)
        self.position = np.array([new_x, new_y])

    def visualize_map(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap="gray_r", interpolation="nearest")
        plt.colorbar(label="Occupancy (0: Free, 1: Occupied)")
        plt.plot(self.position[0], self.position[1], "ro", markersize=10)
        plt.title("Environment Map")
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.show()


if __name__ == "__main__":
    car = AutonomousCar()

    # Simulate multiple scans and movements
    for _ in range(5):
        car.scan_environment()
        car.move(10, np.random.uniform(0, 360))  # Move randomly

    car.visualize_map()

    print(
        f"Percentage of explored area: {np.sum(car.map > 0) / car.map.size * 100:.2f}%"
    )
    print(f"Number of detected obstacles: {np.sum(car.map == 1)}")
