import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1152, 648)  # Set the resolution
picam2.preview_configuration.main.format = "RGB888"  # Set the format
picam2.preview_configuration.align()  # Align the configuration
picam2.configure("preview")  # Set to preview mode
picam2.start()  # Start capturing

# Define the range of orange color in HSV
# You might need to tweak these values to match your specific lighting conditions and camera settings
lower_orange = np.array([5, 150, 150])  # Lower bound of orange (Hue, Saturation, Value)
upper_orange = np.array([15, 255, 255])  # Upper bound of orange (Hue, Saturation, Value)

while True:
    # Capture an image from the camera
    im = picam2.capture_array()

    # Convert the image from RGB to HSV
    hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)

    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply the mask to the original image (optional)
    result = cv2.bitwise_and(im, im, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If any contours are found
    if contours:
        # Find the largest contour (most likely the orange object)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw the bounding box on the image
        cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box

        # Optionally, draw the center of the object
        cx = x + w // 2
        cy = y + h // 2
        cv2.circle(im, (cx, cy), 5, (0, 0, 255), -1)  # Red circle in the center

    # Display the original image with the tracking information
    cv2.imshow("camera feed", im)

    # Press 'q' to quit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
