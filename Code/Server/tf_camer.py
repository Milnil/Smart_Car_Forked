from picamera2 import Picamera2
import numpy as np
import tensorflow as tf
import cv2

# Initialize the PiCamera2
picam2 = Picamera2()
picam2.start()

# Load the TFLite model
interpreter = tf.lite.Interpreter(model_path="efficientdet_lite0.tflite")
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

while True:
    # Capture an image from the camera
    frame = picam2.capture_array()

    # Preprocess the frame (resize, normalize, etc.)
    input_shape = input_details[0]['shape']
    frame_resized = cv2.resize(frame, (input_shape[1], input_shape[2]))
    input_data = np.expand_dims(frame_resized, axis=0)
    input_data = input_data.astype(np.float32)

    # Set the tensor to the model
    interpreter.set_tensor(input_details[0]['index'], input_data)

    # Run inference
    interpreter.invoke()

    # Get the output from the model
    output_data = interpreter.get_tensor(output_details[0]['index'])
    print("Model output:", output_data)

    # Show the frame with OpenCV
    cv2.imshow("PiCamera Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.close()
