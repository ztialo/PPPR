import time
from ball_detection import detect_ball  # Import the ball detection function
import threading
from queue import Queue

# Queue to communicate between threads
coord_queue = Queue()

# Start the ball detection in a separate thread
ball_detection_thread = threading.Thread(target=detect_ball, args=(coord_queue,))
ball_detection_thread.daemon = True  # Allow the thread to exit when the main program exits
ball_detection_thread.start()

last_time = time.time()
# Main loop to get coordinates in real-time
while True:
    if not coord_queue.empty():
        ball_coordinates = coord_queue.get()  # Get the latest coordinates from the Queue

        x, y = ball_coordinates
        if ball_coordinates:
            if last_time - time.time() >= 0.5:
                if x < 110:
                    print("ball at: LL")
                elif x < 220:
                    print("ball at: L")
                elif x < 330:
                    print("ball at: C")
                elif x < 440:
                    print("ball at: R")
                else :
                    print("ball at: RR")

            # Use the coordinates for further processing, e.g., move the robot
            # For example:
            # control_robot(ball_coordinates)
    
    time.sleep(0.1)  # Add a small delay to control the loop speed
