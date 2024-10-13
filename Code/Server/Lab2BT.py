import socket
import threading
import time
from collections import deque
import signal
import logging
import select
from Motor import *
from PCA9685 import PCA9685
from gpiozero import CPUTemperature
from ADC import *
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from PIL import Image
import io

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class CombinedCar:
    def __init__(self, server_addr='D8:3A:DD:F8:70:E3', server_port=5):
        logging.info("Initializing CombinedCar class")
        # Initialize GPIO and sensors
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

        # Bluetooth server setup
        self.server_addr = server_addr
        self.server_port = server_port
        self.server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server_sock.bind((self.server_addr, self.server_port))
        self.server_sock.listen(1)
        logging.info(f"Bluetooth server listening on {self.server_addr}:{self.server_port}")
        print(f"Bluetooth server listening on {self.server_addr}:{self.server_port}")
        
        self.adc = Adc()
        # Initialize Picamera2
        self.picam2 = Picamera2()
        self.configure_camera()
        self.direction = "stopped"
        self.command_map = {
            '87': 'w',  # "W" key for moving forward
            '83': 's',  # "S" key for moving backward
            '65': 'a',  # "A" key for turning left
            '68': 'd',  # "D" key for turning right
            '0': 'stop'
        }

    def configure_camera(self):
        logging.info("Configuring camera")
        video_config = self.picam2.create_still_configuration(main={"size": (320, 240)})
        self.picam2.configure(video_config)
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
        Power = self.adc.recvADC(2)
        battery_life = str(Power * 3)
        direction = self.direction
        temperature = self.get_temperature()
        distance = self.get_distance()
        car_status = f"{direction},{temperature},{distance},{battery_life}"
        logging.debug(f"Car status: {car_status}")
        return car_status

    def handle_drive_command(self, command):
        if command == "w":
            logging.info("Moving forward")
            self.direction = "forward"
            self.PWM.setMotorModel(1000, 1000, 1000, 1000)
        elif command == "s":
            logging.info("Moving backward")
            self.direction = "backward"
            self.PWM.setMotorModel(-1000, -1000, -1000, -1000)
        elif command == "a":
            logging.info("Turning left")
            self.direction = "left"
            self.PWM.setMotorModel(-500, -500, 1500, 1500)
        elif command == "d":
            logging.info("Turning right")
            self.direction = "right"
            self.PWM.setMotorModel(1500, 1500, -500, -500)
        elif command == "stop":
            logging.info("Stopping")
            self.direction = "stopped"
            self.PWM.setMotorModel(0, 0, 0, 0)
        else:
            logging.warning(f"Unknown command: {command}")
            self.PWM.setMotorModel(0, 0, 0, 0)

    def capture_image(self):
        logging.debug("Capturing image")
        try:
            image = self.picam2.capture_array()
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
        logging.info("Waiting for a Bluetooth connection...")
        sock, address = self.server_sock.accept()
        logging.info(f"Connected to {address}")
        sock.setblocking(0)
        message_queue = deque()

        while True:
            try:
                # Receive data
                data = ""
                try:
                    data = sock.recv(1024).decode('utf-8')
                except socket.error:
                    pass  # Non-blocking socket, continue if no data
                
                if data:
                    logging.debug(f"Received data: {data}")
                    if data in self.command_map:
                        self.handle_drive_command(self.command_map[data])

                # Send car status and image
                car_status = self.get_car_status()
                image_data = self.capture_image()
                if image_data is None:
                    image_data = b""
                    image_size = 0
                else:
                    image_size = len(image_data)

                header = f"{car_status}\n{image_size}\n\n"
                sock.sendall(header.encode('utf-8') + image_data)
            except Exception as e:
                logging.error(f"Error in Bluetooth connection: {e}")
                break

        sock.close()
        logging.info("Bluetooth connection closed")
        self.cleanup()

    def cleanup(self):
        logging.info("Cleaning up resources")
        self.PWM.setMotorModel(0, 0, 0, 0)
        self.picam2.stop()
        self.server_sock.close()
        GPIO.cleanup()

# Main program logic follows:
if __name__ == "__main__":
    logging.info("Program is starting...")
    car = CombinedCar()
    try:
        car.run()
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt caught, exiting program")
        car.cleanup()
