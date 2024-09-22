import numpy as np
import math
import time
import matplotlib.pyplot as plt
from queue import PriorityQueue
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
        self.goal = (250, 250) # (horizontal), (vertical)
        self.update_interval = 10  # Update map every 5 steps
        self.additional_padding = 5
        self.detector = ObjectDetector()

        # Create output folder for visualizations
        self.output_folder = "mappings/Routing"
        if os.path.exists(self.output_folder):
            shutil.rmtree(self.output_folder)
        os.makedirs(self.output_folder)

        # # Create mock environment with some obstacles
        # self.create_mock_environment()

        # Initialize mock components
        self.ultrasonic = Ultrasonic()
        self.servo = Servo()
        self.motor = Motor()

    def drive_straight(self, distance_cm):
        """Drive the car straight for a specified distance in cm."""
        speed = 600  # Adjust the speed based on your car's configuration
        duration = distance_cm * 0.05  # Adjust this factor based on your car's speed
        print(f"Driving straight for {distance_cm} cm...")
        self.motor.setMotorModel(speed, speed, speed, speed)
        time.sleep(duration)  # Move forward for the calculated duration
        self.motor.setMotorModel(0, 0, 0, 0)  # Stop the car

    def turn_right(self, duration=1):
        """Turn the car 90 degrees to the right."""
        print("Turning right...")
        self.motor.setMotorModel(2000, 2000, -2000, -2000)  # Rotate right
        time.sleep(duration)  # Adjust time for a 90-degree turn
        self.motor.setMotorModel(0, 0, 0, 0)  # Stop rotation

    def turn_left(self, duration=1):
        """Turn the car 90 degrees to the right."""
        print("Turning left...")
        self.motor.setMotorModel(-2000, -2000, 2000, 2000)  # Rotate right
        time.sleep(duration)  # Adjust time for a 90-degree turn
        self.motor.setMotorModel(0, 0, 0, 0)  # Stop rotation

    def drive_sequence(self):
        """Drive the car straight, turn right, then drive straight again."""
        
        self.look_around()
        # Driving into box 1
        distance = ultrasonic.get_distance()
        while distance == 0:
                distance = ultrasonic.get_distance
        print(f"Driving {distance} cm")
        self.drive_straight(distance-20)  # Drive straight for 50 cm
        self.look_around()
        

        
        self.turn_right(.9)        # Turn right 90 degrees
        self.look_around()
        
        distance = ultrasonic.get_distance()
        while distance == 0:
                distance = ultrasonic.get_distance
        print(f"Driving {distance} cm")

        self.drive_straight(25) # Drive straight again
        self.look_around()
        
        self.turn_left(1.15)
        self.look_around()

        # Driving into box 2
        distance = ultrasonic.get_distance()
        while distance == 0:
                distance = ultrasonic.get_distance
        print(f"Driving {distance} cm")

        self.drive_straight(distance-20)
        self.look_around()
        
        self.turn_left(1.15)
        self.look_around()
        
        distance = ultrasonic.get_distance()
        while distance == 0:
                distance = ultrasonic.get_distance
        print(f"Driving {distance} cm")

        self.drive_straight(40)
        self.look_around()
        
        self.turn_right(.9)
        self.look_around()
        
        distance = ultrasonic.get_distance()
        while distance == 0:
                distance = ultrasonic.get_distance
        print(f"Driving {distance} cm")

        self.drive_straight(80)

    def look_around(self):
        self.servo.setServoPwm("1", 90)
        time.sleep(.2)
        scan_angles = range(0, 180, 10)  
        self.servo.setServoPwm("0", 90)
        time.sleep(0.2)

        for angle in scan_angles:
            self.servo.setServoPwm("0", angle)
            time.sleep(0.2)
        self.servo.setServoPwm("0", 90)
        time.sleep(0.2)

# Example usage
if __name__ == "__main__":
    try:
        mapper = Mapping()
        print("\nExecuting drive sequence...")
        mapper.drive_sequence()  # Execute the sequence of driving straight, turning, and driving again

        print("\nScript Completed.")
    except:
        mapper.motor.setMotorModel(0, 0, 0, 0)  # Stop the car

