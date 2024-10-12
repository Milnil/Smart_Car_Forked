import time
import socket
from Motor import *
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
#from Led import Led  # Import the Led class
import random  # To simulate the temperature data
from gpiozero import CPUTemperature


class CombinedCar:
    def __init__(self, host="192.168.10.59", port=65432):
        # Initialize GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # Initialize ultrasonic sensor pins
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300
        self.timeOut = self.MAX_DISTANCE * 60
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        # Initialize line tracking sensor pins
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)
        # Initialize Motor
        self.PWM = Motor()
        self.M = 0
        # Initialize LED
        #self.led = Led()
        # Server setup
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self.direction = None
        print(f"Server listening on {self.host}:{self.port}")

    def pulseIn(self, pin, level, timeOut):
        t0 = time.time()
        while GPIO.input(pin) != level:
            if (time.time() - t0) > timeOut * 0.000001:
                return 0
        t0 = time.time()
        while GPIO.input(pin) == level:
            if (time.time() - t0) > timeOut * 0.000001:
                return 0
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime

    def get_distance(self):
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0
        distance_cm = sorted(distance_cm)
        return int(distance_cm[2])

    def get_temperature(self):
        cpu = CPUTemperature()
        return cpu if cpu else 0.0

    def get_car_status(self):
        # Get the current car status including direction, temperature, and distance
        direction = self.direction
        temperature = self.get_temperature()
        distance = self.get_distance() if self.get_distance else 0.0
        return f"{direction}, {temperature}, {distance}"

    def handle_drive_command(self, command):
        # Handle the driving commands based on w/a/s/d
        if command == 'w':  # Move forward
            print("Moving forward")
            self.direction = "forward"
            self.PWM.setMotorModel(1000, 1000, 1000, 1000)
        elif command == 's':  # Move backward
            print("Moving backward")
            self.direction = "backward"
            self.PWM.setMotorModel(-1000, -1000, -1000, -1000)
        elif command == 'a':  # Turn left
            print("Turning left")
            self.direction = "left"
            self.PWM.setMotorModel(-500, -500, 1500, 1500)
        elif command == 'd':  # Turn right
            print("Turning right")
            self.direction = "right"
            self.PWM.setMotorModel(1500, 1500, -500, -500)
        elif command == 'stop':
            print("Stopping")
            self.direction = "stopped"
            self.PWM.setMotorModel(0, 0, 0, 0)
        else:
            print(f"Unknown command: {command}")
            # Stop the car if the command is not recognized
            self.PWM.setMotorModel(0, 0, 0, 0)

    def run(self):
        # Accept a connection from the client
        client_socket, client_info = self.server_socket.accept()
        print("Connected by", client_info)
        try:
            while True:

                # Receive data from the client
                data = client_socket.recv(1024).decode().strip().lower()

                print(f"Received command: {data}")

                command_map = {
                    '87': 'w',  # "W" key for moving forward
                    '83': 's',  # "S" key for moving backward
                    '65': 'a',  # "A" key for turning left
                    '68': 'd',   # "D" key for turning right
                    '0': 'stop'
                }
                
                # If the data is one of w/a/s/d, handle driving commands
                if data:
                    self.handle_drive_command(command_map[data])
                else:
                    print(data)
                    print("Client disconnected or sent invalid data.")
                    self.PWM.setMotorModel(0, 0, 0, 0)  # Stop if invalid command

                # Send car status to the connected client
                car_status = self.get_car_status()
                print(f'Car_status = {car_status}')
                try:
                    if car_status:
                        client_socket.sendall(car_status.encode('utf-8'))
                    print(f"Sent to client: {car_status}")
                except BrokenPipeError:
                    print("Client disconnected, broken pipe")
                    break  # Break out of the loop to stop the server from sending data

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            print("Closing connection and cleaning up.")
            client_socket.close()
            self.cleanup()


    def cleanup(self):
        self.PWM.setMotorModel(0, 0, 0, 0)
        #self.led.colorWipe(self.led.strip, Color(0, 0, 0), 10)
        GPIO.cleanup()


# Main program logic follows:
if __name__ == "__main__":
    car = CombinedCar()
    print("Program is starting ... ")
    try:
        car.run()
    except KeyboardInterrupt:
        car.cleanup()
