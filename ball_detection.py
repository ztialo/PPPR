import cv2
import numpy as np
import threading
from picamera2 import Picamera2
import time
from queue import Queue

# Function to initialize the camera and detect the ball
def detect_ball(q: Queue):
    # Initialize Picamera2
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (576, 324)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    # Define the HSV range for white color (with tolerance for greyness)
    lower_white = np.array([0, 0, 200])  # Lower bound: Hue=0 (white), Low saturation, High value
    upper_white = np.array([150, 40, 255])  # Upper bound: Slightly higher hue range (0-10), low saturation, high value

    while True:
        # Capture image from Picamera2
        im = picam2.capture_array()
        im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV
        hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)  # Convert the image to HSV

        # Step 1: Convert image to grayscale for contour detection
        gray = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2GRAY)

        # Step 2: Apply GaussianBlur to reduce noise and improve contour detection
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)

        # Step 3: Use HoughCircles to detect circular objects in the image
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=50, param2=30, minRadius=10, maxRadius=50)

        largest_contour = None
        largest_radius = 0
        center_x, center_y = None, None

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")  # Convert to integer
            for (x, y, radius) in circles:
                # Step 4: Check if the detected circle is large enough and approximately circular
                # Prioritize the largest circle (closest to the camera)
                if radius > largest_radius:
                    largest_radius = radius
                    center_x, center_y = x, y

            # Step 5: Check if the object at (center_x, center_y) is white
            if center_x is not None and center_y is not None:
                hsv_value = hsv[center_y, center_x]  # Get the HSV value at the center of the detected object
                hue_value = hsv_value[0]
                saturation_value = hsv_value[1]
                value_value = hsv_value[2]

                # Check if the center of the object is close to white (HSV)
                if lower_white[0] <= hue_value <= upper_white[0] and lower_white[1] <= saturation_value <= upper_white[1] and lower_white[2] <= value_value <= upper_white[2]:
                    # Send the coordinates to the Queue for real-time access
                    q.put((center_x, center_y))

                    # Draw the largest detected circle and its center
                    cv2.circle(im_bgr, (center_x, center_y), int(largest_radius), (0, 255, 0), 2)  # Green circle
                    cv2.circle(im_bgr, (center_x, center_y), 5, (0, 0, 255), -1)  # Red circle at the center

        # Optionally, you can show the feed for debugging
        cv2.imshow("White Ball Detection", im_bgr)

        # Exit on 'q' key
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
