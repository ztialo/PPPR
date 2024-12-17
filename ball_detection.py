import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1152, 648)  # Set the resolution
picam2.preview_configuration.main.format = "RGB888"  # Set the format to RGB
picam2.preview_configuration.align()  # Align the configuration
picam2.configure("preview")  # Set to preview mode
picam2.start()  # Start capturing

# Define the range of orange color in HSV (may need adjustment)
lower_orange = np.array([5, 150, 150])  # Lower bound of orange (Hue, Saturation, Value)
upper_orange = np.array([15, 255, 255])  # Upper bound of orange (Hue, Saturation, Value)

# Debug: Print the HSV range being used
print(f"Lower Orange HSV: {lower_orange}")
print(f"Upper Orange HSV: {upper_orange}")

while True:
    # Capture an image from the camera
    im = picam2.capture_array()

    # Convert the image from RGB (PiCamera2 format) to BGR (OpenCV format)
    im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

    # Convert the image from BGR to HSV (Hue, Saturation, Value)
    hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

    # Create a mask to detect orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply the mask to the original image (optional, for visual feedback)
    result = cv2.bitwise_and(im_bgr, im_bgr, mask=mask)

    # Debug: Show the mask to verify the color detection
    cv2.imshow("Mask", mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If any contours are found
    if contours:
        # Find the largest contour (most likely the orange object)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding box of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw a green rectangle around the detected object
        cv2.rectangle(im_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle

        # Optionally, draw the center of the object
        cx = x + w // 2
        cy = y + h // 2
        cv2.circle(im_bgr, (cx, cy), 5, (0, 0, 255), -1)  # Red circle in the center

    # Display the processed image with the bounding box and center
    cv2.imshow("Camera Feed", im_bgr)

    # Press 'q' to quit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
