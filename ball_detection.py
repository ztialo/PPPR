import cv2
import numpy as np

# Initialize the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)

    # Threshold to get a binary image
    _, thresholded = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # If the approximation has 4 vertices (a circle), it is a round object
        if len(approx) > 8:  # Circles will have more than 8 vertices
            # Get the center and radius of the circle
            (x, y), radius = cv2.minEnclosingCircle(contour)

            # If the radius is above a threshold, it's likely a ping pong ball
            if radius > 10:
                # Draw the circle on the frame
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow("Ping Pong Ball Detection", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
