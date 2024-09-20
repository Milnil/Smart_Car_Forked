from picamera2 import Picamera2

# Initialize the Picamera2
picam2 = Picamera2()

# Configure the camera (optional, based on your needs)
picam2.configure("preview")  # You can adjust this based on your desired configuration

# Start the camera
picam2.start()

# Capture 30 images
for i in range(30):
    filename = f"images/image_{i}.jpg"  # Define the filename for each image
    picam2.capture_file(filename)  # Capture and save the image

# Stop the camera after capturing
picam2.stop()
