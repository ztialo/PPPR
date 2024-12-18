import cv2
import numpy as np
from picamera2 import Picamera2

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

    # Step 1: Create a mask to isolate the white regions
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Step 2: Find contours in the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Ignore small contours (adjust the size threshold as needed)
        if cv2.contourArea(contour) < 100:
            continue

        # Step 3: Get the minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        
        # Step 4: Check if the detected contour is approximately circular
        aspect_ratio = cv2.contourArea(contour) / (np.pi * (radius ** 2))  # Circularity check
        if aspect_ratio > 0.7:  # Adjust threshold for circularity (closer to 1 is more circular)
            # Draw the circle around the detected object
            cv2.circle(im_bgr, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Green circle

            # Optionally, draw the center of the circle (red)
            cv2.circle(im_bgr, (int(x), int(y)), 5, (0, 0, 255), -1)  # Red circle at the center

            # Get the HSV value at the center of the circle
            hsv_value = hsv[int(y), int(x)]  # Get the HSV value at the center of the detected object
            hue_value = hsv_value[0]
            saturation_value = hsv_value[1]
            value_value = hsv_value[2]

            # Check if the center of the object is close to white (HSV)
            if lower_white[0] <= hue_value <= upper_white[0] and lower_white[1] <= saturation_value <= upper_white[1] and lower_white[2] <= value_value <= upper_white[2]:
                print(f"Detected a white ball at (x, y): ({x}, {y}), Radius: {radius}")

    # Show the result with detected white ball
    cv2.imshow("White Ball Detection", im_bgr)

    # Show the mask
    cv2.imshow("Mask", mask)

    # Exit on 'q' key
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
