1. Attempted to reduce resolution/camera size in order to allow for higher fps at the risk of accuracy but fps didn't go up by a significant margin to warrant it. (Stuck at around 2 fps)

2. Attempted to use multithreading but fps didn't go up by a significant amount. (stuck at slightly above 2 fps)

- Since the Raspberry Pi 4's CPU isn't especially high end, the overhead of using threads seemingly equalizes the benefit provided by threading
- The application of threading was as follows:
- Thread 1: Displaying results, Thread 2: Processing frames, Thread 3: Running the model on the frames

3. Depending on the speed of the vehicle, you need to allow for a higher fps for safety reasons even at the risk of detection because the camera needs to be able to recognize object quicker. Although accuracy is also
   extremely important since the car needs to stop when it sees a pedestrian or stop sign but there can be safety measures in place for the car to always stop when in doubt. So seemingly higher fps would always be more important if those safety measures are implemented.
