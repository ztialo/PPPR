import cv2
import numpy as np
from picamera2 import Picamera2
# use bgr to detect white ball

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (576, 324)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Function to detect white round ball
def detect_ball():
    while True:
        # Capture image from Picamera2
        im = picam2.capture_array()
        im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)  # Convert to BGR color space

        # Step 1: Threshold to detect bright (white) regions
        # White color in BGR: all channels (Blue, Green, Red) should be high
        lower_white = np.array([200, 200, 200])  # Lower bound for white (BGR)
        upper_white = np.array([255, 255, 255])  # Upper bound for white (BGR)

        mask = cv2.inRange(im_bgr, lower_white, upper_white)  # Mask for white regions

        # Step 2: Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = None
        largest_radius = 0
        center_x, center_y = None, None

        # Step 3: Loop through contours and prioritize the largest circular object
        for contour in contours:
            # Ignore small contours
            if cv2.contourArea(contour) < 100:
                continue

            # Get the minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)

            # Step 4: Check if the detected contour is approximately circular
            aspect_ratio = cv2.contourArea(contour) / (np.pi * (radius ** 2))  # Circularity check
            if aspect_ratio > 0.4:  # Threshold for circularity (closer to 1 is more circular)
                # Prioritize the largest circle (closest to the camera)
                if radius > largest_radius:
                    largest_radius = radius
                    largest_contour = contour
                    center_x, center_y = int(x), int(y)

        if largest_contour is not None:
            # Step 5: Draw the largest detected circle and its center
            cv2.circle(im_bgr, (center_x, center_y), int(largest_radius), (0, 255, 0), 2)  # Green circle
            cv2.circle(im_bgr, (center_x, center_y), 5, (0, 0, 255), -1)  # Red circle at the center

            # Print the coordinates of the detected ball (if needed)
            print(f"Detected ball at: ({center_x}, {center_y})")

        # Show the result with detected white ball
        cv2.imshow("White Ball Detection", im_bgr)
        cv2.imshow("Mask", mask)

        # Exit on 'q' key
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

# Call the function to start detecting
detect_ball()
