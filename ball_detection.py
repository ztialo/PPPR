import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1152, 648)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Define the HSV range for light blue (hue from 0 to 3)
lower_hue = np.array([0, 150, 150])  # Lower bound of hue (0 to 3 range), adjust saturation and value
upper_hue = np.array([3, 255, 255])  # Upper bound of hue (0 to 3 range), adjust saturation and value

while True:
    # Capture image from Picamera2
    im = picam2.capture_array()
    im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV
    hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)  # Convert the image to HSV

    # Create a mask to detect the hue range from 0 to 3
    mask = cv2.inRange(hsv, lower_hue, upper_hue)

    # Apply the mask to the image
    result = cv2.bitwise_and(im_bgr, im_bgr, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the minimum enclosing circle and check if it's round
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        if radius > 10:  # Minimum size for detection
            # Draw the circle
            cv2.circle(im_bgr, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Green circle

            # Draw the center of the circle
            cv2.circle(im_bgr, (int(x), int(y)), 5, (0, 0, 255), -1)  # Red circle at the center

            # Optionally, print the radius and coordinates of the detected object
            print(f"Object detected at (x, y): ({x}, {y}), Radius: {radius}")

    # Show the result with detected light blue object
    cv2.imshow("Light Blue Object Detection", im_bgr)

    # Exit on 'q' key
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
