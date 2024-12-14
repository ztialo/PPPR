import cv2
import numpy as np

# Start video capture from the camera
cap = cv2.VideoCapture('/dev/video0')  # Change '0' if using an external camera

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    # Read a frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the color range for detecting the ping pong ball
    # Orange ball (adjust values for your lighting)
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([20, 255, 255])

    # Create a mask for the orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours of the detected object
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filter by size to ignore small objects
        if cv2.contourArea(contour) > 500:
            # Draw a bounding circle around the detected ball
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            # Draw the circle on the original frame
            cv2.circle(frame, center, radius, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Ping Pong Ball Detection", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
