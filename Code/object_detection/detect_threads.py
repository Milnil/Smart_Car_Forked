import argparse
import sys
import time
import threading
import queue

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize
from picamera2 import Picamera2

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()

# Queues for thread-safe communication between threads
frame_queue = queue.Queue(maxsize=1)
result_queue = queue.Queue(maxsize=1)

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)  # Lower resolution to improve FPS
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Global variables to stop threads safely
stop_event = threading.Event()

def capture_frames():
    """Capture frames from the camera and put them in the frame queue."""
    while not stop_event.is_set():
        frame = picam2.capture_array()
        if frame_queue.full():
            frame_queue.get()  # Discard the oldest frame if the queue is full
        frame_queue.put(frame)

def run_inference(model: str, max_results: int, score_threshold: float):
    """Run inference on frames from the frame queue and put results in the result queue."""
    global COUNTER, FPS, START_TIME

    # Initialize the object detection model
    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.LIVE_STREAM,
        max_results=max_results,
        score_threshold=score_threshold,
        result_callback=None  # Callback is not used in this thread-based approach
    )
    detector = vision.ObjectDetector.create_from_options(options)

    while not stop_event.is_set():
        try:
            # Get the latest frame from the queue
            frame = frame_queue.get(timeout=1)
        except queue.Empty:
            continue

        # Resize and flip the frame
        frame = cv2.flip(frame, -1)

        # Convert the frame from BGR to RGB as required by the TFLite model
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run object detection using the model
        result = detector.detect(mp_image)

        # Calculate FPS
        COUNTER += 1
        if COUNTER % 10 == 0:
            FPS = 10 / (time.time() - START_TIME)
            START_TIME = time.time()

        # Store the result in the result queue
        if result_queue.full():
            result_queue.get()  # Discard the oldest result if the queue is full
        result_queue.put((frame, result))

    detector.close()

def display_results():
    """Display the processed frames with object detection results."""
    while not stop_event.is_set():
        try:
            # Get the latest frame and detection result from the result queue
            frame, result = result_queue.get(timeout=1)
        except queue.Empty:
            continue

        # Show the FPS on the frame
        fps_text = f'FPS = {FPS:.1f}'
        cv2.putText(frame, fps_text, (24, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)

        # Visualize detection results
        if result:
            frame = visualize(frame, result)

        # Display the frame
        cv2.imshow('object_detection', frame)

        # Stop the program if the ESC key is pressed
        if cv2.waitKey(1) == 27:
            stop_event.set()

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', required=False, default='best.tflite')
    parser.add_argument('--maxResults', help='Max number of detection results.', required=False, default=5)
    parser.add_argument('--scoreThreshold', help='The score threshold of detection results.', required=False, type=float, default=0.25)
    args = parser.parse_args()

    # Start threads
    capture_thread = threading.Thread(target=capture_frames)
    inference_thread = threading.Thread(target=run_inference, args=(args.model, int(args.maxResults), args.scoreThreshold))
    capture_thread.start()
    inference_thread.start()

    try:
        display_results()  # Run the display function in the main thread
    finally:
        stop_event.set()  # Stop threads when exiting
        capture_thread.join()
        inference_thread.join()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
