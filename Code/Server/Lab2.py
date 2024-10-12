import time
import socket
from Motor import *
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
from gpiozero import CPUTemperature
import select
import queue

from picamera2 import Picamera2
import cv2
import logging

import io
from PIL import Image

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class CombinedCar:
    def __init__(self, host="192.168.10.59", port=65434):
        logging.info("Initializing CombinedCar class")
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
        logging.debug("Ultrasonic sensor pins initialized")
        # Initialize line tracking sensor pins
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)
        logging.debug("Line tracking sensor pins initialized")
        # Initialize Motor
        self.PWM = Motor()
        self.M = 0
        logging.debug("Motor initialized")
        # Server setup
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self.direction = "stopped"
        self.command_map = {
            '87': 'w',  # "W" key for moving forward
            '83': 's',  # "S" key for moving backward
            '65': 'a',  # "A" key for turning left
            '68': 'd',  # "D" key for turning right
            '0': 'stop'
        }
        logging.info(f"Server listening on {self.host}:{self.port}")
        print(f"Server listening on {self.host}:{self.port}")

        # Initialize Picamera2
        self.picam2 = Picamera2()
        # Configure the camera
        self.configure_camera()

    def configure_camera(self):
        logging.info("Configuring camera")
        # Create a configuration suitable for video preview
        video_config = self.picam2.create_still_configuration(main={"size": (320, 240)})
        self.picam2.configure(video_config)
        # Start the camera
        self.picam2.start()
        logging.info("Camera started")

    def pulseIn(self, pin, level, timeOut):
        t0 = time.time()
        while GPIO.input(pin) != level:
            if (time.time() - t0) > timeOut * 0.000001:
                logging.warning("pulseIn timeout waiting for level")
                return 0
        t0 = time.time()
        while GPIO.input(pin) == level:
            if (time.time() - t0) > timeOut * 0.000001:
                logging.warning("pulseIn timeout during level")
                return 0
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime

    def get_distance(self):
        logging.debug("Measuring distance")
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0
        distance_cm = sorted(distance_cm)
        average_distance = int(distance_cm[2])
        logging.debug(f"Distance measured: {average_distance} cm")
        return average_distance

    def get_temperature(self):
        cpu_temp = str(CPUTemperature().temperature)
        logging.debug(f"CPU Temperature: {cpu_temp}")
        return cpu_temp

    def get_car_status(self):
        # Get the current car status including direction, temperature, and distance
        direction = self.direction
        temperature = self.get_temperature()
        distance = self.get_distance()
        car_status = f"{direction},{temperature},{distance}"
        logging.debug(f"Car status: {car_status}")
        return car_status

    def handle_drive_command(self, command):
        # Handle the driving commands based on w/a/s/d
        if command == "w":  # Move forward
            logging.info("Moving forward")
            self.direction = "forward"
            self.PWM.setMotorModel(1000, 1000, 1000, 1000)
        elif command == "s":  # Move backward
            logging.info("Moving backward")
            self.direction = "backward"
            self.PWM.setMotorModel(-1000, -1000, -1000, -1000)
        elif command == "a":  # Turn left
            logging.info("Turning left")
            self.direction = "left"
            self.PWM.setMotorModel(-500, -500, 1500, 1500)
        elif command == "d":  # Turn right
            logging.info("Turning right")
            self.direction = "right"
            self.PWM.setMotorModel(1500, 1500, -500, -500)
        elif command == "stop":
            logging.info("Stopping")
            self.direction = "stopped"
            self.PWM.setMotorModel(0, 0, 0, 0)
        else:
            logging.warning(f"Unknown command received: {command}")
            # Stop the car if the command is not recognized
            self.PWM.setMotorModel(0, 0, 0, 0)

    def capture_image(self):
        # Capture image using Picamera2
        logging.debug("Capturing image")
        try:
            image = self.picam2.capture_array()
            # Convert the image array to JPEG bytes
            img = Image.fromarray(image)
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format='JPEG')
            image_bytes = img_byte_arr.getvalue()
            logging.debug(f"Image captured, size: {len(image_bytes)} bytes")
            return image_bytes
        except Exception as e:
            logging.error(f"Error capturing image: {e}")
            return None

    def run(self):
        logging.info("Starting server run loop")
        self.server_socket.setblocking(False)
        inputs = [self.server_socket]
        outputs = []
        message_queues = {}

        while inputs:
            try:
                readable, writable, exceptional = select.select(
                    inputs, outputs, inputs, 0.1
                )
            except Exception as e:
                logging.error(f"Select error: {e}")
                break

            for s in readable:
                if s is self.server_socket:
                    client_socket, client_address = s.accept()
                    logging.info(f"New connection from {client_address}")
                    client_socket.setblocking(False)
                    inputs.append(client_socket)
                    message_queues[client_socket] = queue.Queue()
                else:
                    try:
                        data = s.recv(1024).decode().strip()
                        if data:
                            logging.debug(f"Received data: {data}")
                            if data in self.command_map:
                                self.handle_drive_command(self.command_map[data])
                            elif data == "0":
                                self.PWM.setMotorModel(0, 0, 0, 0)
                                self.direction = "stopped"
                                logging.info("Received stop command")

                            # Prepare car status
                            car_status = self.get_car_status()

                            # Capture image
                            image_data = self.capture_image()
                            if image_data is None:
                                logging.error("Failed to capture image, sending error message")
                                image_data = b""
                                image_size = 0
                            else:
                                image_size = len(image_data)

                            # Prepare header
                            header = f"{car_status}\n{image_size}\n\n"
                            header_bytes = header.encode('utf-8')
                            logging.debug(f"Sending header: {header.strip()}")

                            # Send header and image data
                            message_queues[s].put(header_bytes + image_data)

                            if s not in outputs:
                                outputs.append(s)
                        else:
                            # Client disconnected
                            logging.info(f"Client disconnected: {s.getpeername()}")
                            if s in outputs:
                                outputs.remove(s)
                            inputs.remove(s)
                            s.close()
                            del message_queues[s]
                    except Exception as e:
                        logging.error(f"Error handling client data: {e}")
                        if s in outputs:
                            outputs.remove(s)
                        inputs.remove(s)
                        s.close()
                        del message_queues[s]

            for s in writable:
                try:
                    next_msg = message_queues[s].get_nowait()
                except queue.Empty:
                    outputs.remove(s)
                else:
                    try:
                        s.sendall(next_msg)
                        logging.debug(f"Sent data to {s.getpeername()}")
                    except Exception as e:
                        logging.error(f"Send error: {e}")
                        if s in outputs:
                            outputs.remove(s)
                        inputs.remove(s)
                        s.close()
                        del message_queues[s]

            for s in exceptional:
                logging.error(f"Exception on {s.getpeername()}")
                inputs.remove(s)
                if s in outputs:
                    outputs.remove(s)
                s.close()
                del message_queues[s]

        self.cleanup()

    def cleanup(self):
        logging.info("Cleaning up resources")
        self.PWM.setMotorModel(0, 0, 0, 0)
        self.picam2.stop()
        GPIO.cleanup()
        logging.info("Cleanup complete")

# Main program logic follows:
if __name__ == "__main__":
    logging.info("Program is starting...")
    car = CombinedCar()
    print("Program is starting ... ")
    try:
        car.run()
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt caught, exiting program")
        car.cleanup()
