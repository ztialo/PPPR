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

# Define the HSV range for light blue (adjust these as necessary)
lower_light_blue = np.array([90, 100, 180])  # Lower bound of light blue
upper_light_blue = np.array([120, 150, 255])  # Upper bound of light blue

while True:
    im = picam2.capture_array()
    im_bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)

    # Create a mask to detect light blue color
    mask = cv2.inRange(hsv, lower_light_blue, upper_light_blue)

    # Apply the mask to the image
    result = cv2.bitwise_and(im_bgr, im_bgr, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(im_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle

        # Optionally, draw the center of the object
        cx = x + w // 2
        cy = y + h // 2
        cv2.circle(im_bgr, (cx, cy), 5, (0, 0, 255), -1)  # Red circle in the center

        # Get the hue value of the object in the center of the bounding box
        hsv_value = hsv[cy, cx]  # Get the HSV value at the center of the object
        hue_value = hsv_value[0]  # Extract the Hue component (first element)
        print(f"Hue Value: {hue_value}")

    # Show the result
    cv2.imshow("Light Blue Tracking", im_bgr)

    # Exit on 'q' key
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
